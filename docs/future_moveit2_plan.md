# Future ROS2 / MoveIt2 Integration Plan (not implemented yet)

This document sketches how the current Python/DYNAMIXEL-SDK-only system
could be extended to ROS2 + MoveIt2 in a later stage. Nothing in this file
is implemented in the current codebase.

## Current stage vs. future stage

| Concern            | Current stage                          | Future stage                     |
|---------------------|------------------------------------------|-----------------------------------|
| Motion planning     | joint-space quintic PTP (`trajectory.py`) | MoveIt2 OMPL/Cartesian planners   |
| Kinematics          | none (degree/raw conversion only)        | URDF-based FK/IK via MoveIt2      |
| Communication       | `DXLDriver` (DYNAMIXEL SDK)               | `ros2_control` hardware interface |
| Trajectory execution| `PTPController` streaming GroupSyncWrite  | `joint_trajectory_controller`     |
| Visualization       | none                                      | RViz                              |
| Description         | none                                      | URDF/Xacro                        |

## Planned extension path

1. **URDF**: Author a URDF/Xacro description of the arm's links and joints,
   matching the physical geometry (link lengths, joint axes, limits already
   present in `calibration.json`).
2. **ros2_control hardware interface**: Wrap `DXLDriver` (or a close
   derivative) as a `ros2_control` `SystemInterface` — `read()` maps to
   `read_joint_deg()`/GroupSyncRead, `write()` maps to
   `sync_write_goal_deg()`/GroupSyncWrite. `HOME_RAW`/`JOINT_SIGN` conversion
   logic in `kinematics.py` is reused as-is.
3. **joint_trajectory_controller**: Replace `PTPController`'s manual
   trajectory streaming with the standard ROS2 controller, which consumes
   `JointTrajectory` messages — the quintic profile in `trajectory.py` can
   still inform trajectory generation if planning bypasses MoveIt2 for
   simple joint-space moves.
3b. Keep `pid.py`'s bounded corrector concept available as an optional
    post-planning trim layer if `ros2_control`'s own loop proves
    insufficient for this hardware.
4. **MoveIt2 config**: Generate a MoveIt2 config package (via the MoveIt
   Setup Assistant or `moveit_configs_utils`) from the URDF, enabling
   Cartesian pose planning and collision-aware motion.
5. **RViz**: Visualize planning scenes and executed trajectories.
6. **Gripper (id 7) and 6th joint (id 6)**: Both are already structurally
   separated in this codebase (`gripper_id` in `arm_config.yaml`, index-
   aligned config lists everywhere else) so wiring them in — physically and
   in URDF — does not require refactoring existing code.

## What carries over unchanged

- `config/calibration.json` (home_raw, joint_sign, joint limits) — feeds
  both the current raw driver and a future `ros2_control` hardware
  interface identically.
- `src/dxl_arm/kinematics.py` degree/raw conversion — reused inside the
  hardware interface's `read()`/`write()`.
- `src/dxl_arm/safety.py` — checks remain valid regardless of what issues
  the joint commands.
