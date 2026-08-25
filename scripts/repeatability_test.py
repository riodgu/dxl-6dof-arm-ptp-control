"""Repeat a home -> target -> home cycle N times and report positioning repeatability.

Usage:
    python scripts/repeatability_test.py --target 10 20 -10 5 0 --repeat 50 --time 3.0
"""

import argparse
import time

import numpy as np
import pandas as pd

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config, load_calibration, validate_config_compatibility
from dxl_arm.dxl_driver import DXLDriver
from dxl_arm.logger import DEFAULT_LOG_DIR
from dxl_arm.pid import default_pid_gains
from dxl_arm.ptp_controller import PTPController


def main():
    parser = argparse.ArgumentParser(description="Measure joint-space PTP repeatability.")
    parser.add_argument("--target", type=float, nargs="+", required=True, help="target joint angles in degrees")
    parser.add_argument("--repeat", type=int, default=10, help="number of repetitions")
    parser.add_argument("--time", type=float, default=None, help="move time in seconds")
    args = parser.parse_args()

    arm_config = load_arm_config()
    calibration = load_calibration()
    validate_config_compatibility(arm_config, calibration)
    move_time = args.time if args.time is not None else arm_config.default_move_time
    num_joints = len(arm_config.dxl_ids)

    if len(args.target) != num_joints:
        raise SystemExit(f"expected {num_joints} joint angles, got {len(args.target)}")

    home_deg = calibration.home_deg

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
        pid_gains=default_pid_gains(num_joints),
    )

    rows = []
    driver.connect()
    try:
        driver.setup_motors()
        controller.move_ptp_joint(home_deg, move_time=move_time)

        for trial in range(args.repeat):
            print(f"Trial {trial + 1}/{args.repeat}...")
            t0 = time.time()
            final_deg = controller.move_ptp_joint(args.target, move_time=move_time)
            reach_time = time.time() - t0

            error = [t - f for t, f in zip(args.target, final_deg)]
            row = {"trial": trial + 1, "reach_time_s": reach_time}
            for i in range(num_joints):
                row[f"final_j{i + 1}"] = final_deg[i]
                row[f"error_j{i + 1}"] = error[i]
            rows.append(row)

            controller.move_ptp_joint(home_deg, move_time=move_time)

        df = pd.DataFrame(rows)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = f"{DEFAULT_LOG_DIR}/repeatability_{timestamp}.csv"
        df.to_csv(out_path, index=False)
        print(f"\nSaved raw results to {out_path}")

        print("\nRepeatability summary:")
        for i in range(num_joints):
            errors = df[f"error_j{i + 1}"].to_numpy()
            print(
                f"J{i + 1} mean error: {np.mean(np.abs(errors)):.2f} deg  "
                f"max error: {np.max(np.abs(errors)):.2f} deg  "
                f"std: {np.std(errors):.2f} deg"
            )
    except KeyboardInterrupt:
        print("\nInterrupted, disabling torque...")
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
