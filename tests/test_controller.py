"""Hardware-free tests for controller safety, timing, logging, and results."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import dxl_arm.ptp_controller as controller_module
from dxl_arm.logger import CSVLogger
from dxl_arm.pid import PIDGains, default_pid_gains
from dxl_arm.ptp_controller import PTPController
from dxl_arm.safety import EmergencyStop


GAINS = [PIDGains(1.0, 0.0, 0.0, 10.0, 10.0)]


class FakeDriver:
    def __init__(self, position=0.0):
        self.position = position
        self.commands = []
        self.torque_disabled = False
        self.temperature = [25.0]
        self.voltage = [12.0]
        self.hardware_error = [0]
        self.fail_writes = False

    def read_joint_deg(self):
        return [self.position]

    def sync_write_goal_deg(self, goal):
        if self.fail_writes:
            raise RuntimeError("simulated write failure")
        self.commands.append(list(goal))

    def read_temperature(self):
        return self.temperature

    def read_voltage(self):
        return self.voltage

    def read_hardware_error(self):
        return self.hardware_error

    def disable_torque(self):
        self.torque_disabled = True


def make_controller(driver, **kwargs):
    return PTPController(driver, [-30.0], [100.0], GAINS, **kwargs)


def test_pid_commands_are_clamped_to_joint_limits():
    driver = FakeDriver(position=99.0)
    controller = make_controller(driver)

    controller.correct_position_pid(
        [100.0], timeout=0.01, hz=1000.0, tolerance_deg=0.0
    )

    assert driver.commands
    assert all(-30.0 <= command[0] <= 100.0 for command in driver.commands)


def test_no_pid_returns_measured_position():
    driver = FakeDriver(position=7.5)
    controller = make_controller(driver)

    result = controller.move_ptp_joint([10.0], move_time=0.001, hz=1000.0, use_pid=False)

    assert result == [7.5]


@pytest.mark.parametrize(
    ("attribute", "unsafe_value", "message"),
    (
        ("temperature", [80.0], "temperature"),
        ("voltage", [8.0], "voltage"),
        ("hardware_error", [1], "hardware error"),
    ),
)
def test_unsafe_motor_status_disables_torque_before_motion(
    attribute, unsafe_value, message
):
    driver = FakeDriver()
    setattr(driver, attribute, unsafe_value)
    controller = make_controller(driver)

    with pytest.raises(EmergencyStop, match=message):
        controller.move_ptp_joint([10.0], move_time=0.001, hz=1000.0, use_pid=False)

    assert driver.torque_disabled
    assert driver.commands == []


def test_log_is_closed_when_motion_write_fails(tmp_path):
    driver = FakeDriver()
    driver.fail_writes = True
    logger = CSVLogger(str(tmp_path))
    controller = make_controller(driver, logger=logger)

    with pytest.raises(RuntimeError, match="simulated write failure"):
        controller.move_ptp_joint(
            [10.0], move_time=0.001, hz=1000.0, use_pid=False, log_filename="motion"
        )

    assert logger._file is None


def test_trajectory_scheduler_does_not_sleep_after_last_waypoint(monkeypatch):
    class FakeClock:
        value = 0.0

        def monotonic(self):
            return self.value

        def sleep(self, seconds):
            self.value += seconds

    clock = FakeClock()
    monkeypatch.setattr(controller_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(controller_module.time, "sleep", clock.sleep)

    driver = FakeDriver()
    controller = make_controller(driver, safety_check_interval=100.0)
    controller.move_ptp_joint([10.0], move_time=1.0, hz=4.0, use_pid=False)

    assert clock.value == pytest.approx(1.0)
    assert len(driver.commands) == 5


def test_default_pid_gains_support_six_joints():
    assert len(default_pid_gains(6)) == 6


def test_continuous_sequence_rejects_later_invalid_pose_before_motion():
    driver = FakeDriver()
    controller = make_controller(driver)

    with pytest.raises(ValueError, match="pose 2"):
        controller.move_continuous_joint_sequence(
            [[10.0], [101.0]], segment_times=[0.1, 0.1], use_pid_final=False
        )

    assert driver.commands == []
