"""Safety checks: joint limits, temperature, voltage, and hardware errors."""

from typing import Sequence, Tuple


def check_joint_limits(
    angles: Sequence[float],
    min_limits: Sequence[float],
    max_limits: Sequence[float],
) -> Tuple[bool, str]:
    """Reject any commanded angle set that violates a joint's min/max limit.

    Returns (is_safe, message).
    """
    if not (len(angles) == len(min_limits) == len(max_limits)):
        return False, "joint limit check: length mismatch between angles and limits"

    for i, angle in enumerate(angles):
        if angle < min_limits[i] or angle > max_limits[i]:
            return False, (
                f"joint {i} angle {angle:.2f} deg exceeds limit "
                f"[{min_limits[i]:.2f}, {max_limits[i]:.2f}]"
            )
    return True, ""


def check_temperature(temperatures: Sequence[float], max_temp: float = 70) -> Tuple[bool, str]:
    """Check every motor temperature (deg C) is at or below max_temp."""
    for i, temp in enumerate(temperatures):
        if temp > max_temp:
            return False, f"joint {i} temperature {temp:.1f} C exceeds max {max_temp:.1f} C"
    return True, ""


def check_voltage(
    voltages: Sequence[float],
    min_voltage: float = 10.0,
    max_voltage: float = 14.0,
) -> Tuple[bool, str]:
    """Check every motor input voltage is within [min_voltage, max_voltage]."""
    for i, voltage in enumerate(voltages):
        if voltage < min_voltage or voltage > max_voltage:
            return False, (
                f"joint {i} voltage {voltage:.2f} V out of range "
                f"[{min_voltage:.2f}, {max_voltage:.2f}]"
            )
    return True, ""


def check_hardware_error(errors: Sequence[int]) -> Tuple[bool, str]:
    """Check that no motor reports a nonzero hardware error status byte."""
    for i, err in enumerate(errors):
        if err != 0:
            return False, f"joint {i} hardware error status = 0x{err:02X}"
    return True, ""


class EmergencyStop(Exception):
    """Raised to signal that motion must halt and torque must be disabled."""


def trigger_emergency_stop(driver, reason: str) -> None:
    """Disable torque on the driver and raise EmergencyStop with reason.

    Any caller in the motion loop can invoke this the moment a safety check
    (limits, temperature, voltage, hardware error) fails.
    """
    try:
        driver.disable_torque()
    finally:
        raise EmergencyStop(reason)
