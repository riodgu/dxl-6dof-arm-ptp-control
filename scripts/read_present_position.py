"""Read and print present raw position and its degree conversion for all joints.

Usage:
    python scripts/read_present_position.py
"""

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config, load_calibration, validate_config_compatibility
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
        raw_positions = driver.read_joint_raw()
        deg_positions = driver.joint_raw_values_to_deg(raw_positions)

        print(f"{'joint':>6} {'id':>4} {'raw':>10} {'deg':>10}")
        for i, dxl_id in enumerate(arm_config.dxl_ids):
            print(f"{i:>6} {dxl_id:>4} {raw_positions[i]:>10} {deg_positions[i]:>10.2f}")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
