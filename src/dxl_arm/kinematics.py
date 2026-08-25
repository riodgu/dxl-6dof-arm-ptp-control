"""Joint-space degree <-> raw pulse conversion and limit checking.

No forward/inverse kinematics here by design — this project stage only
needs per-joint degree/raw conversion plus limit validation. FK/IK are
deferred to a future ROS2/MoveIt2 stage (see docs/future_moveit2_plan.md).
"""

from typing import Sequence, Tuple


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value to the inclusive [minimum, maximum] range."""
    return max(minimum, min(maximum, value))


def validate_joint_angles(
    angles: Sequence[float],
    joint_min_deg: Sequence[float],
    joint_max_deg: Sequence[float],
) -> Tuple[bool, str]:
    """Check that every angle is within its joint's [min, max] limits.

    Returns (is_valid, message). message is empty when valid, otherwise it
    names the first offending joint.
    """
    if not (len(angles) == len(joint_min_deg) == len(joint_max_deg)):
        return False, (
            f"length mismatch: angles={len(angles)}, "
            f"min={len(joint_min_deg)}, max={len(joint_max_deg)}"
        )

    for i, angle in enumerate(angles):
        if angle < joint_min_deg[i] or angle > joint_max_deg[i]:
            return False, (
                f"joint {i} angle {angle:.2f} deg out of range "
                f"[{joint_min_deg[i]:.2f}, {joint_max_deg[i]:.2f}]"
            )
    return True, ""


def joint_deg_to_raw(
    joint_index: int,
    angle_deg: float,
    home_raw: Sequence[int],
    joint_sign: Sequence[int],
    pulse_per_deg: float,
    extended_raw_min: int = -1048575,
    extended_raw_max: int = 1048575,
) -> int:
    """Convert a joint angle in degrees to an absolute raw encoder position.

    raw = home_raw[i] + joint_sign[i] * angle_deg * pulse_per_deg
    """
    raw = home_raw[joint_index] + joint_sign[joint_index] * angle_deg * pulse_per_deg
    raw = int(round(raw))
    raw = int(clamp(raw, extended_raw_min, extended_raw_max))
    return raw


def joint_raw_to_deg(
    joint_index: int,
    raw: int,
    home_raw: Sequence[int],
    joint_sign: Sequence[int],
    deg_per_pulse: float,
) -> float:
    """Convert an absolute raw encoder position back to a joint angle in degrees.

    angle_deg = (raw - home_raw[i]) * deg_per_pulse / joint_sign[i]
    """
    return (raw - home_raw[joint_index]) * deg_per_pulse / joint_sign[joint_index]
