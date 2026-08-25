"""Ping every motor listed in arm_config.yaml and report success/failure.

Usage:
    python scripts/ping_motors.py
"""

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config
from dxl_arm.dxl_driver import DXLDriver


def main():
    arm_config = load_arm_config()

    driver = DXLDriver(
        device_name=arm_config.device_name,
        baudrate=arm_config.baudrate,
        protocol_version=arm_config.protocol_version,
        dxl_ids=arm_config.dxl_ids,
        home_raw=[0] * len(arm_config.dxl_ids),
        joint_sign=[1] * len(arm_config.dxl_ids),
    )

    driver.connect()
    try:
        print(f"Pinging {len(arm_config.dxl_ids)} motor(s) on {arm_config.device_name}...")
        all_ok = True
        for dxl_id in arm_config.dxl_ids:
            ok = driver.ping(dxl_id)
            status = "OK" if ok else "FAILED"
            print(f"  id {dxl_id}: {status}")
            all_ok = all_ok and ok

        if all_ok:
            print("All motors responded.")
        else:
            print("One or more motors did not respond. Check wiring/power/ID.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
