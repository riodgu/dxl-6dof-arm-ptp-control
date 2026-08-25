"""Tests for degree/raw conversion and joint limit validation."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dxl_arm.kinematics import joint_deg_to_raw, joint_raw_to_deg, validate_joint_angles

HOME_RAW = (2051, 834, 1019, 2032, 2664)
JOINT_SIGN = (1, 1, -1, 1, 1)
PULSE_PER_DEG = 4096 / 360.0
DEG_PER_PULSE = 360.0 / 4096


def test_deg_to_raw_positive_sign():
    raw = joint_deg_to_raw(0, 10.0, HOME_RAW, JOINT_SIGN, PULSE_PER_DEG)
    expected = HOME_RAW[0] + 1 * 10.0 * PULSE_PER_DEG
    assert raw == round(expected)


def test_deg_to_raw_zero_is_home():
    raw = joint_deg_to_raw(0, 0.0, HOME_RAW, JOINT_SIGN, PULSE_PER_DEG)
    assert raw == HOME_RAW[0]


def test_raw_to_deg_zero_at_home():
    angle = joint_raw_to_deg(0, HOME_RAW[0], HOME_RAW, JOINT_SIGN, DEG_PER_PULSE)
    assert angle == 0.0


def test_round_trip_positive_sign():
    # Round-trip is only accurate to within one encoder pulse (~0.088 deg),
    # since joint_deg_to_raw rounds to the nearest integer raw position.
    original_deg = 25.0
    raw = joint_deg_to_raw(0, original_deg, HOME_RAW, JOINT_SIGN, PULSE_PER_DEG)
    back_deg = joint_raw_to_deg(0, raw, HOME_RAW, JOINT_SIGN, DEG_PER_PULSE)
    assert abs(back_deg - original_deg) < DEG_PER_PULSE


def test_negative_joint_sign():
    # joint index 2 has JOINT_SIGN = -1
    joint_index = 2
    angle_deg = 15.0
    raw = joint_deg_to_raw(joint_index, angle_deg, HOME_RAW, JOINT_SIGN, PULSE_PER_DEG)
    expected_raw = HOME_RAW[joint_index] + (-1) * angle_deg * PULSE_PER_DEG
    assert raw == round(expected_raw)

    back_deg = joint_raw_to_deg(joint_index, raw, HOME_RAW, JOINT_SIGN, DEG_PER_PULSE)
    assert abs(back_deg - angle_deg) < DEG_PER_PULSE


def test_negative_joint_sign_direction_inverted():
    # For JOINT_SIGN = -1, a positive angle should move raw DOWN from home.
    joint_index = 2
    raw = joint_deg_to_raw(joint_index, 10.0, HOME_RAW, JOINT_SIGN, PULSE_PER_DEG)
    assert raw < HOME_RAW[joint_index]


def test_validate_joint_angles_within_limits():
    angles = [0, 10, -10, 5, 0]
    min_deg = [-30, -30, -30, -30, -30]
    max_deg = [120, 100, 100, 80, 80]
    ok, msg = validate_joint_angles(angles, min_deg, max_deg)
    assert ok
    assert msg == ""


def test_validate_joint_angles_exceeds_max():
    angles = [0, 200, -10, 5, 0]
    min_deg = [-30, -30, -30, -30, -30]
    max_deg = [120, 100, 100, 80, 80]
    ok, msg = validate_joint_angles(angles, min_deg, max_deg)
    assert not ok
    assert "joint 1" in msg


def test_validate_joint_angles_below_min():
    angles = [-50, 10, -10, 5, 0]
    min_deg = [-30, -30, -30, -30, -30]
    max_deg = [120, 100, 100, 80, 80]
    ok, msg = validate_joint_angles(angles, min_deg, max_deg)
    assert not ok
    assert "joint 0" in msg
