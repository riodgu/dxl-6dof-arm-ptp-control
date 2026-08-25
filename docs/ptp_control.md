# PTP (Point-to-Point) Control

## Overview

Every move in this project is a joint-space PTP move: given a target set of
joint angles, the controller generates a smooth trajectory from the current
pose to the goal and streams it to the motors.

## Trajectory shape: why quintic

`src/dxl_arm/trajectory.py` supports `linear`, `cubic`, and `quintic`
interpolation ratios, but the default method used everywhere
(`PTPController.move_ptp_joint`) is `quintic`:

```
s(r) = 10*r^3 - 15*r^4 + 6*r^5
```

- `linear`: constant velocity, but infinite acceleration at both endpoints
  (a velocity step) — causes a mechanical jerk at the start and stop of
  every move.
- `cubic` (`s = 3r^2 - 2r^3`): velocity is zero at both endpoints, but
  acceleration is not — there is still a jerk discontinuity.
- `quintic`: both velocity AND acceleration are zero at `r=0` and `r=1`.
  This gives the smoothest start/stop profile of the three, which reduces
  mechanical stress and backlash-induced oscillation on DYNAMIXEL servos.

## Move sequence (`PTPController.move_ptp_joint`)

1. Validate the goal has the right number of joints.
2. Validate the goal is within `joint_min_deg` / `joint_max_deg`
   (`safety.check_joint_limits`); the move is rejected if not.
3. Read the current joint angles (`DXLDriver.read_joint_deg`, GroupSyncRead).
4. Generate a quintic trajectory from current -> goal over `move_time`
   seconds at `hz` waypoints/sec.
5. Stream each waypoint to the motors with
   `DXLDriver.sync_write_goal_deg` (GroupSyncWrite), sleeping `1/hz`
   between scheduled steps. Temperature, voltage, and hardware-error status
   are checked periodically; an unsafe reading disables torque.
6. If `use_pid=True`, run `correct_position_pid()` to remove residual
   steady-state error.
7. Return the final measured joint angles.
8. If a `log_filename` is given, every streamed waypoint's target/actual/
   error angles are written to `data/logs/<log_filename>.csv`.

## PID correction pass

DYNAMIXEL's built-in Extended Position Control Mode already closes its own
position loop, so `correct_position_pid()` is a light final-error trim, not
a full control loop: it runs at a lower rate (`hz=20` by default) for up to
`timeout` seconds, commanding `goal_deg + pid_output`, and exits early once
every joint's error has stayed within `tolerance_deg` for `stable_samples`
consecutive cycles.

The PID-adjusted command is clamped to each joint's configured minimum and
maximum. This prevents a valid goal near a limit from being pushed beyond the
limit by the correction term.

## Command-line usage

```bash
# One-shot move
python scripts/move_once.py 10 20 -10 5 0 --time 3.0

# Sequence of poses
python scripts/run_sequence.py

# Move out and back N times, measuring repeatability
python scripts/repeatability_test.py --target 10 20 -10 5 0 --repeat 50 --time 3.0
```
