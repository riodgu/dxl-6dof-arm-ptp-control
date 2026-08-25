"""Loading of arm_config.yaml and calibration.json into typed structures."""

import json
import os
from dataclasses import dataclass, asdict
from typing import List, Optional

import yaml

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_ARM_CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "arm_config.yaml")
DEFAULT_CALIBRATION_PATH = os.path.join(PROJECT_ROOT, "config", "calibration.json")


@dataclass
class ArmConfig:
    """Static hardware / communication configuration for the arm."""

    device_name: str
    baudrate: int
    protocol_version: float
    dxl_ids: List[int]
    gripper_id: int
    operating_mode: int
    profile_acceleration: int
    profile_velocity: int
    control_hz: float
    default_move_time: float


@dataclass
class Calibration:
    """Per-joint calibration data. All lists are index-aligned with dxl_ids."""

    home_raw: List[int]
    home_deg: List[float]
    joint_sign: List[int]
    joint_min_deg: List[float]
    joint_max_deg: List[float]


def load_yaml(path: str) -> dict:
    """Load a YAML file and return its contents as a dict."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_json(path: str) -> dict:
    """Load a JSON file and return its contents as a dict."""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: str, data: dict) -> None:
    """Write a dict to a JSON file with stable, human-readable formatting."""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def load_arm_config(path: Optional[str] = None) -> ArmConfig:
    """Load arm_config.yaml (default: config/arm_config.yaml) into an ArmConfig."""
    path = path or DEFAULT_ARM_CONFIG_PATH
    raw = load_yaml(path)
    return ArmConfig(
        device_name=raw["device_name"],
        baudrate=int(raw["baudrate"]),
        protocol_version=float(raw["protocol_version"]),
        dxl_ids=list(raw["dxl_ids"]),
        gripper_id=int(raw["gripper_id"]),
        operating_mode=int(raw["operating_mode"]),
        profile_acceleration=int(raw["profile_acceleration"]),
        profile_velocity=int(raw["profile_velocity"]),
        control_hz=float(raw["control_hz"]),
        default_move_time=float(raw["default_move_time"]),
    )


def load_calibration(path: Optional[str] = None) -> Calibration:
    """Load calibration.json (default: config/calibration.json) into a Calibration."""
    path = path or DEFAULT_CALIBRATION_PATH
    raw = load_json(path)
    calibration = Calibration(
        home_raw=list(raw["home_raw"]),
        home_deg=list(raw["home_deg"]),
        joint_sign=list(raw["joint_sign"]),
        joint_min_deg=list(raw["joint_min_deg"]),
        joint_max_deg=list(raw["joint_max_deg"]),
    )
    lengths = {len(getattr(calibration, field)) for field in calibration.__dataclass_fields__}
    if len(lengths) != 1:
        raise ValueError("all calibration lists must have the same length")
    if any(sign not in (-1, 1) for sign in calibration.joint_sign):
        raise ValueError("joint_sign entries must be either -1 or 1")
    if any(lo > hi for lo, hi in zip(calibration.joint_min_deg, calibration.joint_max_deg)):
        raise ValueError("each joint_min_deg must be <= joint_max_deg")
    return calibration


def validate_config_compatibility(arm_config: ArmConfig, calibration: Calibration) -> None:
    """Validate that every per-joint calibration list matches ``dxl_ids``."""
    num_joints = len(arm_config.dxl_ids)
    if num_joints == 0:
        raise ValueError("dxl_ids must not be empty")
    if len(set(arm_config.dxl_ids)) != num_joints:
        raise ValueError("dxl_ids must be unique")
    for field in calibration.__dataclass_fields__:
        size = len(getattr(calibration, field))
        if size != num_joints:
            raise ValueError(
                f"calibration.{field} has {size} entries, expected {num_joints}"
            )


def save_calibration(calibration: Calibration, path: Optional[str] = None) -> None:
    """Persist a Calibration back to calibration.json (default path)."""
    path = path or DEFAULT_CALIBRATION_PATH
    save_json(path, asdict(calibration))
