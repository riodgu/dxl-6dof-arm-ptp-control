"""High-level point-to-point (PTP) joint-space motion controller.

Combines trajectory generation, GroupSyncWrite streaming, PID correction,
safety checks, and optional CSV logging into a single control API.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, List, Optional, Sequence

from .logger import CSVLogger
from .pid import JointPID, PIDGains
from .safety import (
    check_hardware_error,
    check_joint_limits,
    check_temperature,
    check_voltage,
    trigger_emergency_stop,
)
from .trajectory import generate_continuous_joint_trajectory, generate_joint_trajectory

if TYPE_CHECKING:
    from .dxl_driver import DXLDriver


class PTPController:
    """Drives the arm through quintic PTP moves with optional PID correction."""

    def __init__(
        self,
        driver: DXLDriver,
        joint_min_deg: Sequence[float],
        joint_max_deg: Sequence[float],
        pid_gains: Sequence[PIDGains],
        logger: Optional[CSVLogger] = None,
        safety_check_interval: float = 0.5,
        max_temperature: float = 70.0,
        min_voltage: float = 10.0,
        max_voltage: float = 14.0,
    ):
        self.driver = driver
        self.joint_min_deg = list(joint_min_deg)
        self.joint_max_deg = list(joint_max_deg)
        self.num_joints = len(joint_min_deg)

        if self.num_joints == 0:
            raise ValueError("at least one joint is required")
        if len(joint_max_deg) != self.num_joints:
            raise ValueError("joint_min_deg and joint_max_deg lengths must match")
        if any(lo > hi for lo, hi in zip(self.joint_min_deg, self.joint_max_deg)):
            raise ValueError("each joint minimum must be <= its maximum")
        if len(pid_gains) != self.num_joints:
            raise ValueError("pid_gains length must match number of joints")
        if safety_check_interval <= 0:
            raise ValueError("safety_check_interval must be positive")
        self._pids = [JointPID(g) for g in pid_gains]

        self.logger = logger
        self.safety_check_interval = safety_check_interval
        self.max_temperature = max_temperature
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self._next_safety_check = 0.0

    # ---------------------------------------------------------------- #

    def get_current_joint_deg(self) -> List[float]:
        """Read and return the current joint angles in degrees."""
        return self.driver.read_joint_deg()

    def move_ptp_joint(
        self,
        goal_deg: Sequence[float],
        move_time: float = 3.0,
        hz: float = 100.0,
        use_pid: bool = True,
        pid_timeout: float = 3.0,
        tolerance_deg: float = 1.0,
        log_filename: Optional[str] = None,
    ) -> List[float]:
        """Move to goal_deg via a quintic joint-space trajectory.

        Steps: validate goal count and limits -> read current pose ->
        generate quintic trajectory -> stream via sync_write_goal_deg ->
        optional PID correction at the end -> return final joint angles.
        """
        if len(goal_deg) != self.num_joints:
            raise ValueError(
                f"goal_deg has {len(goal_deg)} joints, expected {self.num_joints}"
            )

        ok, msg = check_joint_limits(goal_deg, self.joint_min_deg, self.joint_max_deg)
        if not ok:
            raise ValueError(f"move_ptp_joint rejected: {msg}")

        start_deg = self.get_current_joint_deg()
        trajectory = generate_joint_trajectory(start_deg, goal_deg, move_time, hz, method="quintic")

        logging_active = log_filename is not None
        if logging_active:
            if self.logger is None:
                raise RuntimeError("log_filename given but no CSVLogger was configured")
            self.logger.start_log(log_filename, num_joints=self.num_joints)

        dt = move_time / (len(trajectory) - 1)
        schedule_start = time.monotonic()
        self._next_safety_check = 0.0
        try:
            for index, waypoint in enumerate(trajectory):
                if index:
                    remaining = schedule_start + index * dt - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                self._check_runtime_safety()
                self.driver.sync_write_goal_deg(waypoint)
                if logging_active:
                    actual = self.get_current_joint_deg()
                    error = [target - measured for target, measured in zip(waypoint, actual)]
                    self.logger.write_motion_sample(time.time(), waypoint, actual, error)

            if use_pid:
                return self.correct_position_pid(
                    goal_deg, timeout=pid_timeout, tolerance_deg=tolerance_deg
                )
            return self.get_current_joint_deg()
        finally:
            if logging_active:
                self.logger.close()

    def correct_position_pid(
        self,
        goal_deg: Sequence[float],
        timeout: float = 3.0,
        hz: float = 20.0,
        tolerance_deg: float = 1.0,
        stable_samples: int = 3,
    ) -> List[float]:
        """Run bounded per-joint PID correction until within tolerance.

        Applies goal_deg + pid_output as the commanded angle each cycle.
        Stops early once every joint's error stays within tolerance_deg for
        stable_samples consecutive cycles, or when timeout is reached.
        """
        if len(goal_deg) != self.num_joints:
            raise ValueError(f"goal_deg has {len(goal_deg)} joints, expected {self.num_joints}")
        ok, msg = check_joint_limits(goal_deg, self.joint_min_deg, self.joint_max_deg)
        if not ok:
            raise ValueError(f"correct_position_pid rejected: {msg}")
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        if hz <= 0:
            raise ValueError("hz must be positive")
        if tolerance_deg < 0:
            raise ValueError("tolerance_deg must be non-negative")
        if stable_samples <= 0:
            raise ValueError("stable_samples must be positive")

        for pid in self._pids:
            pid.reset()

        dt = 1.0 / hz
        deadline = time.time() + timeout
        consecutive_stable = 0
        current_deg = self.get_current_joint_deg()

        while time.time() < deadline:
            self._check_runtime_safety()
            current_deg = self.get_current_joint_deg()
            errors = [g - c for g, c in zip(goal_deg, current_deg)]

            if all(abs(e) <= tolerance_deg for e in errors):
                consecutive_stable += 1
                if consecutive_stable >= stable_samples:
                    break
            else:
                consecutive_stable = 0

            corrections = [pid.update(e, dt) for pid, e in zip(self._pids, errors)]
            commanded = [
                max(lo, min(hi, goal + correction))
                for goal, correction, lo, hi in zip(
                    goal_deg, corrections, self.joint_min_deg, self.joint_max_deg
                )
            ]
            self.driver.sync_write_goal_deg(commanded)

            time.sleep(dt)

        return current_deg

    def _check_runtime_safety(self) -> None:
        """Periodically stop motion if a motor reports an unsafe condition."""
        now = time.monotonic()
        if now < self._next_safety_check:
            return
        self._next_safety_check = now + self.safety_check_interval

        checks = (
            lambda: check_temperature(self.driver.read_temperature(), self.max_temperature),
            lambda: check_voltage(
                self.driver.read_voltage(), self.min_voltage, self.max_voltage
            ),
            lambda: check_hardware_error(self.driver.read_hardware_error()),
        )
        for check in checks:
            safe, message = check()
            if not safe:
                trigger_emergency_stop(self.driver, message)

    def move_and_return_home(
        self,
        goal_deg: Sequence[float],
        home_deg: Sequence[float],
        move_time: float = 3.0,
        home_time: Optional[float] = None,
        dwell_time: float = 1.0,
    ) -> List[float]:
        """Move to goal_deg, dwell, then move back to home_deg. Returns final angles."""
        home_time = home_time if home_time is not None else move_time
        self.move_ptp_joint(goal_deg, move_time=move_time)
        time.sleep(dwell_time)
        return self.move_ptp_joint(home_deg, move_time=home_time)

    def move_continuous_joint_sequence(
        self,
        goals_deg: Sequence[Sequence[float]],
        segment_times: Sequence[float],
        hz: float = 100.0,
        use_pid_final: bool = True,
    ) -> List[float]:
        """Pass through multiple joint poses without stopping at each pose."""
        if not goals_deg:
            raise ValueError("at least one goal pose is required")
        if len(segment_times) != len(goals_deg):
            raise ValueError("one segment time is required for each goal pose")

        for pose_index, goal in enumerate(goals_deg, start=1):
            if len(goal) != self.num_joints:
                raise ValueError(
                    f"pose {pose_index} has {len(goal)} joints, expected {self.num_joints}"
                )
            ok, msg = check_joint_limits(goal, self.joint_min_deg, self.joint_max_deg)
            if not ok:
                raise ValueError(f"continuous sequence pose {pose_index} rejected: {msg}")

        start_deg = self.get_current_joint_deg()
        poses = [start_deg] + [list(goal) for goal in goals_deg]
        trajectory = generate_continuous_joint_trajectory(poses, segment_times, hz)

        # Validate generated samples as well as captured poses before motion starts.
        for waypoint in trajectory:
            ok, msg = check_joint_limits(waypoint, self.joint_min_deg, self.joint_max_deg)
            if not ok:
                raise ValueError(f"continuous trajectory rejected: {msg}")

        dt = sum(segment_times) / (len(trajectory) - 1)
        schedule_start = time.monotonic()
        self._next_safety_check = 0.0
        for index, waypoint in enumerate(trajectory):
            if index:
                remaining = schedule_start + index * dt - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            self._check_runtime_safety()
            self.driver.sync_write_goal_deg(waypoint)

        if use_pid_final:
            return self.correct_position_pid(goals_deg[-1])
        return self.get_current_joint_deg()

    def run_sequence(self, sequence: Sequence[dict], default_move_time: float = 3.0) -> List[List[float]]:
        """Run a list of {"name", "q", "time", "dwell"} pose dicts in order.

        Returns the list of final joint angles achieved for each pose.
        """
        results = []
        for pose in sequence:
            name = pose.get("name", "unnamed")
            q = pose["q"]
            move_time = pose.get("time", default_move_time)
            dwell = pose.get("dwell", 0.0)

            print(f"[PTPController] moving to '{name}': {q}")
            final = self.move_ptp_joint(q, move_time=move_time)
            results.append(final)

            if dwell > 0:
                time.sleep(dwell)

        return results
