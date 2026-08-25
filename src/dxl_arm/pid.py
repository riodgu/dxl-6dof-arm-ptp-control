"""Bounded per-joint PID corrector used for final position error correction."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PIDGains:
    """Gains and safety bounds for a single joint's PID corrector."""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float


class JointPID:
    """A single-joint PID controller with bounded integral and output."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def reset(self) -> None:
        """Clear integral and derivative history."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def update(self, error: float, dt: float) -> float:
        """Compute one PID output step for the given error and time delta.

        The integral term is clamped to +-integral_limit (anti-windup) and
        the final output is clamped to +-output_limit.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        g = self.gains

        self._integral += error * dt
        self._integral = max(-g.integral_limit, min(g.integral_limit, self._integral))

        derivative = 0.0
        if self._has_prev:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._has_prev = True

        output = g.kp * error + g.ki * self._integral + g.kd * derivative
        output = max(-g.output_limit, min(g.output_limit, output))
        return output


# Default per-joint gains, index-aligned with dxl_ids (joints 1-5).
PID_GAINS = (
    PIDGains(kp=0.60, ki=0.03, kd=0.01, integral_limit=10.0, output_limit=6.0),
    PIDGains(kp=0.90, ki=0.08, kd=0.02, integral_limit=10.0, output_limit=10.0),
    PIDGains(kp=1.00, ki=0.12, kd=0.02, integral_limit=10.0, output_limit=12.0),
    PIDGains(kp=0.60, ki=0.03, kd=0.01, integral_limit=10.0, output_limit=6.0),
    PIDGains(kp=0.60, ki=0.03, kd=0.01, integral_limit=10.0, output_limit=6.0),
)


def default_pid_gains(num_joints: int):
    """Return conservative PID gains for any supported joint count.

    The tuned gains above are retained for joints 1-5. Additional joints use
    the conservative wrist-joint defaults until hardware-specific tuning is
    supplied.
    """
    if num_joints <= 0:
        raise ValueError("num_joints must be positive")
    if num_joints <= len(PID_GAINS):
        return PID_GAINS[:num_joints]
    return PID_GAINS + (PID_GAINS[-1],) * (num_joints - len(PID_GAINS))
