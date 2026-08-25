# DYNAMIXEL 6-DOF Robot Arm PTP Control System

## Overview

A Python-based joint-space point-to-point (PTP) control system for a
DYNAMIXEL-based 6-DOF robot arm. It uses only the DYNAMIXEL SDK and Python
standard tooling — no ROS2, URDF, RViz, or MoveIt2 at this stage (see
`docs/future_moveit2_plan.md` for the planned extension path).

## Features

- DYNAMIXEL SDK communication
- Joint degree/raw conversion
- HOME calibration
- Joint direction sign calibration
- Quintic joint-space trajectory
- GroupSyncRead / GroupSyncWrite
- Bounded PID correction
- Repeatability test
- CSV logging
- Teach & Replay
- Future ROS2 / MoveIt2 integration (planned, not implemented)

## Hardware

- DXL430 x 5
- DXL330 x 1 for 6th joint
- DXL330 x 1 for gripper
- U2D2
- TTL connection
- Windows PC, `COM3`
- Baudrate 1,000,000, Protocol 2.0

Currently only motors 1-5 are controlled (`dxl_ids: [1, 2, 3, 4, 5]` in
`config/arm_config.yaml`). Every config list and library function is
index-aligned with `dxl_ids`, so extending to 6-DOF is a configuration
change, not a code change — see `docs/system_overview.md`.

## Project layout

```text
dxl-6dof-arm-ptp-control/
├── config/            arm_config.yaml, calibration.json
├── src/dxl_arm/         control_table, config, dxl_driver, kinematics,
│                          trajectory, pid, ptp_controller, logger, safety
├── scripts/            CLI entry points (see below)
├── data/logs/         CSV logs written by CSVLogger
├── docs/                system, calibration, PTP, PID, future-MoveIt2 docs
└── tests/              pytest unit tests
```

## Quick Start

Install dependencies:

```bash
pip install -r requirements.txt
```

Check motor communication:

```bash
python scripts/ping_motors.py
```

Read current position (raw + degrees):

```bash
python scripts/read_present_position.py
```

Calibrate HOME position:

```bash
python scripts/calibrate_home.py
```

Calibrate joint rotation direction:

```bash
python scripts/test_joint_sign.py
```

Move once to a target pose:

```bash
python scripts/move_once.py 10 20 -10 5 0 --time 3.0
```

Run a predefined sequence of poses:

```bash
python scripts/run_sequence.py
```

Measure repeatability over N trials:

```bash
python scripts/repeatability_test.py --target 10 20 -10 5 0 --repeat 50 --time 3.0
```

Teach poses by hand, then replay them:

```bash
python scripts/teach_and_replay.py
```

Run the test suite:

```bash
pytest
```

## Calibration order

1. `calibrate_home.py` — sets `home_raw` so degree 0 matches your chosen
   physical HOME pose.
2. `test_joint_sign.py` — sets `joint_sign` so a positive commanded degree
   always matches the physically-observed positive rotation direction.

See `docs/calibration_guide.md` for full details.

## Safety

- Every commanded joint-space move is checked against
  `joint_min_deg`/`joint_max_deg` in `config/calibration.json` before it is
  sent to the motors (`src/dxl_arm/safety.py`).
- Temperature, voltage, and hardware-error status are checked before motion
  and periodically while trajectory/PID commands are running. An unsafe
  reading disables torque and raises an emergency-stop error.
- `DXLDriver.shutdown()` disables torque before closing the port, and every
  script calls it from a `finally` block so a `KeyboardInterrupt` or
  unexpected exception still leaves the arm detorqued.

## Not implemented in this stage

ROS2 nodes, URDF, RViz, MoveIt2, ros2_control, joint_trajectory_controller,
Cartesian pose control, IK solvers, camera vision, ArUco tracking.

## Docs

- `docs/system_overview.md` — architecture and data flow
- `docs/calibration_guide.md` — HOME and JOINT_SIGN calibration steps
- `docs/ptp_control.md` — trajectory generation and PTP move sequence
- `docs/pid_tuning.md` — PID gain reference and tuning procedure
- `docs/future_moveit2_plan.md` — planned ROS2/MoveIt2 extension path
