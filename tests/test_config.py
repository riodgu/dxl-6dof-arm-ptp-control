"""Tests for cross-file arm and calibration validation."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dxl_arm.config import ArmConfig, Calibration, validate_config_compatibility


def arm(ids):
    return ArmConfig("COM3", 1_000_000, 2.0, ids, 7, 4, 20, 80, 100.0, 3.0)


def calibration(count):
    return Calibration(
        [0] * count,
        [0.0] * count,
        [1] * count,
        [-30.0] * count,
        [30.0] * count,
    )


def test_configuration_rejects_calibration_length_mismatch():
    with pytest.raises(ValueError, match="home_raw"):
        validate_config_compatibility(arm([1, 2]), calibration(1))


def test_six_joint_configuration_is_accepted():
    validate_config_compatibility(arm([1, 2, 3, 4, 5, 6]), calibration(6))
