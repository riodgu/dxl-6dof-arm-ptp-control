"""Move each joint +10 deg one at a time and record the actual sign of rotation.

For each joint: read the current angle, command +10 deg, ask the user
whether the arm moved in the positive direction, then return the joint to
its starting angle. Results are saved into calibration.json's joint_sign.

Usage:
    python scripts/test_joint_sign.py
    python scripts/test_joint_sign.py --exclude 6 7
"""

import argparse

import _pathfix  # noqa: F401

from dxl_arm.config import (
    DEFAULT_CALIBRATION_PATH,
    load_arm_config,
    load_calibration,
    save_calibration,
    validate_config_compatibility,
)
from dxl_arm.dxl_driver import DXLDriver

TEST_STEP_DEG = 10.0


def main():
    parser = argparse.ArgumentParser(description="Test and calibrate JOINT_SIGN per joint.")
    parser.add_argument(
        "--exclude", type=int, nargs="*", default=[],
        help="motor IDs to exclude from the test (e.g. 6th joint / gripper)",
    )
    args = parser.parse_args()

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

    driver.connect()
    try:
        driver.setup_motors()
        new_joint_sign = list(calibration.joint_sign)

        for i, dxl_id in enumerate(arm_config.dxl_ids):
            if dxl_id in args.exclude:
                print(f"Skipping id {dxl_id} (excluded).")
                continue

            print(f"\n--- Testing joint {i} (id {dxl_id}) ---")
            start_deg = driver.read_joint_deg()
            target_deg = list(start_deg)
            target_deg[i] += TEST_STEP_DEG

            driver.sync_write_goal_deg(target_deg)
            input("Press Enter after the motion has stopped...")

            answer = input("Did the joint rotate in the POSITIVE direction? [y/N] ").strip().lower()
            new_joint_sign[i] = 1 if answer == "y" else -1

            driver.sync_write_goal_deg(start_deg)
            input("Press Enter once it has returned to the starting position...")

        calibration.joint_sign = new_joint_sign
        print(f"\nNew joint_sign: {new_joint_sign}")
        answer = input(f"Save to {DEFAULT_CALIBRATION_PATH}? [y/N] ").strip().lower()
        if answer == "y":
            save_calibration(calibration)
            print("Saved.")
        else:
            print("Not saved.")
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
