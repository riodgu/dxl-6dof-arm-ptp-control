# Calibration Guide

Two calibration steps are required before running any PTP motion:
HOME_RAW calibration and JOINT_SIGN calibration. Both are stored in
`config/calibration.json`.

## 1. HOME calibration (`home_raw`)

`home_raw` is the raw encoder position that corresponds to 0 deg for every
joint — the reference point every degree<->raw conversion is measured from.

Steps:

1. Run `python scripts/calibrate_home.py`.
2. The script disables torque so you can move the arm by hand.
3. Move the arm to the desired HOME pose (usually a safe, centered pose).
4. Press Enter. The script reads present raw positions for all joints.
5. Confirm to save — this overwrites `home_raw` in `config/calibration.json`.

`home_deg` stays `[0.0, 0.0, ...]` by convention; it exists so future code
can support a non-zero "home" angle definition without changing the field
name.

## 2. JOINT_SIGN calibration (`joint_sign`)

`joint_sign` corrects for motors that are mechanically mounted so that
increasing raw position corresponds to decreasing joint angle (or vice
versa). It is `+1` or `-1` per joint.

Steps:

1. Run `python scripts/test_joint_sign.py` (add `--exclude 6 7` to skip
   joints not yet wired).
2. For each joint: the script commands +10 deg from the current angle.
3. Observe the physical arm and answer whether it moved in the positive
   direction you expect (e.g. counter-clockwise as viewed from a
   defined reference direction).
4. The script returns the joint to its starting angle before testing the
   next joint.
5. Confirm to save the resulting `joint_sign` list.

## Why calibration must happen in this order

`joint_sign` testing commands relative moves (`current + 10 deg`), which are
converted to raw using the current `home_raw`. Do HOME calibration first so
that the `home_raw` used during sign testing is meaningful — otherwise the
resulting `joint_sign` will still be correct (it's a relative test), but the
readable angle values you see during the test won't reflect the true pose.

## Verifying calibration

After both steps, run:

```bash
python scripts/read_present_position.py
```

With the arm at the HOME pose, all joints should read ~0.0 deg. Manually
move one joint in the positive direction and re-run — the angle should
increase.
