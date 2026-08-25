"""Interactively save the current pose's raw position as home_raw in calibration.json.

Usage:
    python scripts/calibrate_home.py
"""

import _pathfix  # noqa: F401

from dxl_arm.config import (
    DEFAULT_CALIBRATION_PATH,
    load_arm_config,
    load_calibration,
    save_calibration,
    validate_config_compatibility,
)
from dxl_arm.dxl_driver import DXLDriver


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
    )

    driver.connect()
    try:
        driver.disable_torque()
        print("Torque disabled. Manually move the arm to the desired HOME pose.")
        input("Press Enter when the arm is in position...")

        raw_positions = driver.read_joint_raw()
        print("Current raw positions:")
        for i, dxl_id in enumerate(arm_config.dxl_ids):
            print(f"  id {dxl_id} (joint {i}): {raw_positions[i]}")

        answer = input(f"Save these as home_raw in {DEFAULT_CALIBRATION_PATH}? [y/N] ").strip().lower()
        if answer == "y":
            calibration.home_raw = raw_positions
            save_calibration(calibration)
            print("Saved.")
        else:
            print("Not saved.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
