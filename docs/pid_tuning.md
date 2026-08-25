# PID Tuning Guide

## Role of the PID corrector

`src/dxl_arm/pid.py`'s `JointPID` is not the primary position controller —
DYNAMIXEL's Extended Position Control Mode already runs its own internal
position loop. `JointPID` is used by `PTPController.correct_position_pid()`
purely to trim residual steady-state error after a trajectory-based move
finishes, by nudging the commanded goal angle: `commanded = goal + output`.

## Gains

```python
PIDGains(kp, ki, kd, integral_limit, output_limit)
```

- `kp`: proportional gain — the primary corrective force per degree of error.
- `ki`: integral gain — removes small steady-state offsets; kept low to
  avoid windup and oscillation.
- `kd`: derivative gain — damps oscillation as error approaches zero.
- `integral_limit`: hard clamp on the accumulated integral term (anti-windup).
- `output_limit`: hard clamp on the final PID output added to the goal
  angle, in degrees. This is the most important safety bound — it caps how
  far the corrector can push the commanded position away from the planned
  trajectory goal.

Default gains (`PID_GAINS` in `pid.py`), index-aligned with joints 1-5:

| Joint | kp   | ki   | kd   | integral_limit | output_limit |
|-------|------|------|------|-----------------|--------------|
| 1     | 0.60 | 0.03 | 0.01 | 10.0            | 6.0          |
| 2     | 0.90 | 0.08 | 0.02 | 10.0            | 10.0         |
| 3     | 1.00 | 0.12 | 0.02 | 10.0            | 12.0         |
| 4     | 0.60 | 0.03 | 0.01 | 10.0            | 6.0          |
| 5     | 0.60 | 0.03 | 0.01 | 10.0            | 6.0          |

Joints 2 and 3 carry more load (shoulder/elbow) and use higher gains and
output limits than the wrist joints.

## Tuning procedure

1. Start with `ki = kd = 0`, only `kp`. Increase `kp` until the joint
   reliably closes small errors without visibly oscillating.
2. Add a small `kd` if you see overshoot/ringing when approaching the
   target — it damps the approach.
3. Add a small `ki` only if you observe a persistent steady-state offset
   that `kp`/`kd` alone don't remove. Keep it small; a large `ki` combined
   with the low update rate (`hz=20` default) causes wind-up-driven
   oscillation.
4. Always set `output_limit` conservatively — it should be small enough
   that even a fully saturated PID output cannot command a large, sudden
   jump in goal position. A few degrees is a reasonable starting point.
5. Re-run `python scripts/repeatability_test.py` after any gain change and
   compare mean/max error and std across joints.

## Safety notes

- `output_limit` and `integral_limit` are enforced unconditionally in
  `JointPID.update()` — there is no way to bypass them from the caller.
- The final `goal + output` command is also clamped to the configured joint
  limits by `PTPController`, including when the goal itself is on a limit.
- If a joint oscillates persistently, reduce `kp` and/or `output_limit`
  before adding more `kd` — a bounded corrector that undershoots slightly is
  safer than one that hunts around the target.
