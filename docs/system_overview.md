# System Overview

## Purpose

This project implements joint-space point-to-point (PTP) control for a
DYNAMIXEL-based 6-DOF robot arm using only Python and the DYNAMIXEL SDK.
No ROS2, URDF, RViz, or MoveIt2 is used at this stage.

## Hardware

- DXL430 x5 (joints 1-5), DXL330 x1 (joint 6, not yet wired), DXL330 x1 (gripper, id 7, not yet wired)
- U2D2 USB-TTL adapter
- Windows PC, `COM3`, baudrate `1,000,000`, Protocol 2.0
- Operating mode: Extended Position Control Mode (4), chosen because joint
  raw positions may cross the 0-pulse boundary during motion.

## Software architecture

```
config/            static YAML/JSON configuration and calibration data
src/dxl_arm/        library code
  control_table.py  DYNAMIXEL control table addresses/constants
  config.py         loads arm_config.yaml / calibration.json
  kinematics.py      degree <-> raw conversion, limit validation (no FK/IK)
  trajectory.py       quintic/cubic/linear PTP trajectory generation
  pid.py               bounded per-joint PID corrector
  dxl_driver.py        DYNAMIXEL SDK wrapper (single + GroupSync read/write)
  safety.py            limit/temperature/voltage/hw-error checks, e-stop
  logger.py            CSV logging of motion samples and summaries
  ptp_controller.py     ties trajectory + driver + PID + logging together
scripts/            CLI entry points for calibration, motion, testing
tests/              pytest unit tests for conversion and trajectory logic
```

## Data flow for one PTP move

1. `PTPController.move_ptp_joint(goal_deg, ...)` validates the goal against
   `calibration.json`'s joint limits (`safety.check_joint_limits`).
2. Current joint angles are read via `DXLDriver.read_joint_deg()`
   (GroupSyncRead).
3. `trajectory.generate_joint_trajectory()` produces a quintic-interpolated
   waypoint list from the current pose to the goal.
4. Each waypoint is streamed to the motors via
   `DXLDriver.sync_write_goal_deg()` (GroupSyncWrite), paced at `hz`.
5. Optionally, `PTPController.correct_position_pid()` runs a bounded PID
   loop per joint until the final error is within `tolerance_deg`; corrected
   commands remain clamped to the configured joint limits.
6. If a logger is configured, every step's target/actual/error angles are
   appended to a CSV file under `data/logs/`.
7. Temperature, voltage, and hardware-error status are monitored during
   trajectory and PID execution. An unsafe reading disables torque.

## Extending to 6-DOF

Add the 6th joint's DYNAMIXEL id, home_raw, joint_sign, and joint limits to
`config/arm_config.yaml` / `config/calibration.json`. Every list-based
config field and every function in `src/dxl_arm/` is index-aligned with
`dxl_ids`, so no code changes are required — only configuration. Joints beyond
the five tuned PID entries receive conservative default gains until separately
tuned; predefined five-joint sequence poses keep additional joints at HOME.

## Not implemented at this stage

ROS2 nodes, URDF, RViz, MoveIt2, ros2_control, joint_trajectory_controller,
Cartesian pose control, IK solvers, camera vision, ArUco tracking. See
`docs/future_moveit2_plan.md` for the planned extension path.
