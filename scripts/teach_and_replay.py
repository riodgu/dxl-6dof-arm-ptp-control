"""Teach poses by hand (torque off) then replay them as a PTP sequence.

Usage:
    python scripts/teach_and_replay.py
"""

import _pathfix  # noqa: F401

from dxl_arm.config import load_arm_config, load_calibration, validate_config_compatibility
from dxl_arm.dxl_driver import DXLDriver
from dxl_arm.logger import CSVLogger
from dxl_arm.pid import default_pid_gains
from dxl_arm.ptp_controller import PTPController

DEFAULT_MOVE_TIME = 1.0

# Teach/replay uses a wider range than normal task execution. These remain
# software limits; set them to match the arm's actual collision-free envelope.
TEACH_JOINT_MIN_DEG = -360.0
TEACH_JOINT_MAX_DEG = 360.0


def teach(driver: DXLDriver):
    """Torque off; let the user move the arm by hand and record poses on Enter."""
    driver.disable_torque()
    print("Torque disabled. Move the arm by hand.")
    print("Press Enter to record the current pose, or type 'done' to finish.")

    poses = []
    while True:
        cmd = input(f"[pose {len(poses) + 1}] Enter to record / 'done' to finish: ").strip().lower()
        if cmd == "done":
            break
        pose = driver.read_joint_deg()
        poses.append(pose)
        print(f"  recorded: {[round(a, 2) for a in pose]}")

    return poses


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

    num_joints = len(arm_config.dxl_ids)
    controller = PTPController(
        driver=driver,
        joint_min_deg=[TEACH_JOINT_MIN_DEG] * num_joints,
        joint_max_deg=[TEACH_JOINT_MAX_DEG] * num_joints,
        pid_gains=default_pid_gains(num_joints),
        logger=CSVLogger(),
    )

    driver.connect()
    try:
        driver.setup_motors()
        poses = teach(driver)

        if not poses:
            print("No poses recorded. Exiting.")
            return

        print(f"\nRecorded {len(poses)} pose(s). Preparing continuous replay...")
        print(
            f"Teach limits: {TEACH_JOINT_MIN_DEG:.0f} to "
            f"{TEACH_JOINT_MAX_DEG:.0f} deg per joint"
        )

        # Validate every capture before torque is enabled, so replay cannot stop
        # halfway because a later recorded pose exceeds the teach-mode limits.
        for pose_index, pose in enumerate(poses, start=1):
            for joint_index, angle in enumerate(pose, start=1):
                if not TEACH_JOINT_MIN_DEG <= angle <= TEACH_JOINT_MAX_DEG:
                    raise ValueError(
                        f"pose_{pose_index} joint {joint_index} angle {angle:.2f} deg "
                        f"exceeds teach limit [{TEACH_JOINT_MIN_DEG:.2f}, "
                        f"{TEACH_JOINT_MAX_DEG:.2f}]"
                    )

        print("Enabling torque for continuous replay...")
        driver.enable_torque()
        final = controller.move_continuous_joint_sequence(
            poses,
            segment_times=[DEFAULT_MOVE_TIME] * len(poses),
        )
        print(f"Replay complete. Final = {[round(a, 2) for a in final]}")

        print("Returning to HOME...")
        home_final = controller.move_ptp_joint(
            calibration.home_deg,
            move_time=DEFAULT_MOVE_TIME,
        )
        print(f"HOME reached: {[round(a, 2) for a in home_final]}")
        print("Teach & Replay finished. Disabling torque and exiting.")
    except KeyboardInterrupt:
        print("\nInterrupted, disabling torque...")
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
