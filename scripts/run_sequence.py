"""Run a predefined sequence of joint poses in order.

Usage:
    python scripts/run_sequence.py
"""

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config, load_calibration, validate_config_compatibility
from dxl_arm.dxl_driver import DXLDriver
from dxl_arm.logger import CSVLogger
from dxl_arm.pid import default_pid_gains
from dxl_arm.ptp_controller import PTPController

# Edit this sequence to suit your task. "q" length must match dxl_ids length.
SEQUENCE = [
    {"name": "home", "q": [0, 0, 0, 0, 0], "time": 3.0, "dwell": 1.0},
    {"name": "point_a", "q": [10, 20, -10, 5, 0], "time": 3.0, "dwell": 1.0},
    {"name": "point_b", "q": [20, 10, -20, 10, 5], "time": 3.0, "dwell": 1.0},
    {"name": "home", "q": [0, 0, 0, 0, 0], "time": 3.0, "dwell": 1.0},
]


def main():
    arm_config = load_arm_config()
    calibration = load_calibration()
    validate_config_compatibility(arm_config, calibration)

    driver = DXLDriver(
        device_name=arm_config.device_name,
        baudrate=arm_config.baudrate,
        protocol_version=arm_config.protocol_version,
        dxl_ids=arm_config.dxl_ids,
        home_raw=calibration.home_raw,
        joint_sign=calibration.joint_sign,
        operating_mode=arm_config.operating_mode,
        profile_acceleration=arm_config.profile_acceleration,
        profile_velocity=arm_config.profile_velocity,
    )

    controller = PTPController(
        driver=driver,
        joint_min_deg=calibration.joint_min_deg,
        joint_max_deg=calibration.joint_max_deg,
        pid_gains=default_pid_gains(len(arm_config.dxl_ids)),
        logger=CSVLogger(),
    )

    driver.connect()
    try:
        driver.setup_motors()
        sequence = []
        for pose in SEQUENCE:
            if len(pose["q"]) > len(arm_config.dxl_ids):
                raise ValueError(f"pose '{pose['name']}' has too many joint values")
            normalized = dict(pose)
            normalized["q"] = list(pose["q"]) + calibration.home_deg[len(pose["q"]):]
            sequence.append(normalized)
        results = controller.run_sequence(sequence, default_move_time=arm_config.default_move_time)
        for pose, final in zip(sequence, results):
            print(f"{pose['name']}: final = {[round(a, 2) for a in final]}")
    except KeyboardInterrupt:
        print("\nInterrupted, disabling torque...")
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
