"""Move the arm once to a target joint pose given on the command line.

Usage:
    python scripts/move_once.py 10 20 -10 5 0 --time 3.0

When 6-DOF is enabled (dxl_ids extended to 6 entries), pass 6 values:
    python scripts/move_once.py 10 20 -10 5 0 15 --time 3.0
"""

import argparse

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config, load_calibration, validate_config_compatibility
from dxl_arm.dxl_driver import DXLDriver
from dxl_arm.logger import CSVLogger
from dxl_arm.pid import default_pid_gains
from dxl_arm.ptp_controller import PTPController


def main():
    parser = argparse.ArgumentParser(description="Move the arm once to a target joint pose.")
    parser.add_argument("angles", type=float, nargs="+", help="target joint angles in degrees")
    parser.add_argument("--time", type=float, default=None, help="move time in seconds")
    parser.add_argument("--no-pid", action="store_true", help="skip final PID correction")
    args = parser.parse_args()

    arm_config = load_arm_config()
    calibration = load_calibration()
    validate_config_compatibility(arm_config, calibration)
    move_time = args.time if args.time is not None else arm_config.default_move_time

    if len(args.angles) != len(arm_config.dxl_ids):
        raise SystemExit(
            f"expected {len(arm_config.dxl_ids)} joint angles, got {len(args.angles)}"
        )

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
        print(f"Moving to {args.angles} deg over {move_time}s...")
        final_deg = controller.move_ptp_joint(
            args.angles, move_time=move_time, use_pid=not args.no_pid
        )
        print(f"Final joint angles: {[round(a, 2) for a in final_deg]}")
    except KeyboardInterrupt:
        print("\nInterrupted, disabling torque...")
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
