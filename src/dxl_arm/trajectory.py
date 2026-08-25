"""Point-to-point joint-space trajectory generation.

Default interpolation method is "quintic": unlike cubic, its velocity AND
acceleration are both zero at the start and end of the move, which avoids
the acceleration discontinuity (and the mechanical jerk it causes) that a
cubic profile has at t=0 and t=T on a DYNAMIXEL-driven arm.
"""

from typing import List, Sequence


def linear_interpolation_ratio(r: float) -> float:
    """Linear ratio: s(r) = r."""
    return r


def cubic_interpolation_ratio(r: float) -> float:
    """Cubic ratio with zero velocity at endpoints: s(r) = 3r^2 - 2r^3."""
    return 3 * r**2 - 2 * r**3


def quintic_interpolation_ratio(r: float) -> float:
    """Quintic ratio with zero velocity AND acceleration at endpoints.

    s(r) = 10*r^3 - 15*r^4 + 6*r^5
    """
    return 10 * r**3 - 15 * r**4 + 6 * r**5


_METHODS = {
    "linear": linear_interpolation_ratio,
    "cubic": cubic_interpolation_ratio,
    "quintic": quintic_interpolation_ratio,
}


def generate_joint_trajectory(
    start_deg: Sequence[float],
    goal_deg: Sequence[float],
    move_time: float,
    hz: float,
    method: str = "quintic",
) -> List[List[float]]:
    """Generate a joint-space PTP trajectory from start_deg to goal_deg.

    Returns a list of waypoints, each a list of per-joint angles in degrees.
    The first waypoint equals start_deg and the last equals goal_deg.
    """
    if len(start_deg) != len(goal_deg):
        raise ValueError("start_deg and goal_deg must have the same length")
    if move_time <= 0:
        raise ValueError("move_time must be positive")
    if hz <= 0:
        raise ValueError("hz must be positive")
    if method not in _METHODS:
        raise ValueError(f"unknown method '{method}', expected one of {list(_METHODS)}")

    ratio_fn = _METHODS[method]
    num_steps = max(1, int(round(move_time * hz)))

    trajectory: List[List[float]] = []
    for step in range(num_steps + 1):
        t = step / num_steps
        s = ratio_fn(t)
        waypoint = [
            start_deg[j] + s * (goal_deg[j] - start_deg[j])
            for j in range(len(start_deg))
        ]
        trajectory.append(waypoint)

    # Guarantee exact endpoints regardless of floating point interpolation error.
    trajectory[0] = list(start_deg)
    trajectory[-1] = list(goal_deg)
    return trajectory


def generate_continuous_joint_trajectory(
    poses_deg: Sequence[Sequence[float]],
    segment_times: Sequence[float],
    hz: float,
) -> List[List[float]]:
    """Generate a position- and velocity-continuous trajectory through poses.

    A shape-preserving cubic Hermite spline is evaluated independently for
    each joint. Internal poses are crossed with a continuous velocity instead
    of stopping at every pose. Tangents are set to zero at direction changes,
    which prevents spline overshoot around local minima and maxima.

    ``segment_times[i]`` is the duration from pose ``i`` to pose ``i + 1``.
    The first and last velocities are zero.
    """
    if len(poses_deg) < 2:
        raise ValueError("at least two poses are required")
    if len(segment_times) != len(poses_deg) - 1:
        raise ValueError("segment_times length must be len(poses_deg) - 1")
    if any(duration <= 0 for duration in segment_times):
        raise ValueError("all segment times must be positive")
    if hz <= 0:
        raise ValueError("hz must be positive")

    num_joints = len(poses_deg[0])
    if num_joints == 0 or any(len(pose) != num_joints for pose in poses_deg):
        raise ValueError("all poses must have the same non-zero joint count")

    # Per-segment slopes, followed by shape-preserving PCHIP-style tangents.
    slopes = [
        [
            (poses_deg[i + 1][joint] - poses_deg[i][joint]) / segment_times[i]
            for joint in range(num_joints)
        ]
        for i in range(len(segment_times))
    ]
    tangents = [[0.0] * num_joints for _ in poses_deg]
    for i in range(1, len(poses_deg) - 1):
        previous_time = segment_times[i - 1]
        next_time = segment_times[i]
        for joint in range(num_joints):
            previous_slope = slopes[i - 1][joint]
            next_slope = slopes[i][joint]
            if previous_slope == 0 or next_slope == 0 or previous_slope * next_slope <= 0:
                tangents[i][joint] = 0.0
                continue
            weight_1 = 2 * next_time + previous_time
            weight_2 = next_time + 2 * previous_time
            tangents[i][joint] = (weight_1 + weight_2) / (
                weight_1 / previous_slope + weight_2 / next_slope
            )

    trajectory: List[List[float]] = []
    for segment, duration in enumerate(segment_times):
        num_steps = max(1, int(round(duration * hz)))
        # The following segment owns the shared boundary, avoiding duplicates.
        first_step = 0 if segment == 0 else 1
        for step in range(first_step, num_steps + 1):
            u = step / num_steps
            h00 = 2 * u**3 - 3 * u**2 + 1
            h10 = u**3 - 2 * u**2 + u
            h01 = -2 * u**3 + 3 * u**2
            h11 = u**3 - u**2
            waypoint = [
                h00 * poses_deg[segment][joint]
                + h10 * duration * tangents[segment][joint]
                + h01 * poses_deg[segment + 1][joint]
                + h11 * duration * tangents[segment + 1][joint]
                for joint in range(num_joints)
            ]
            trajectory.append(waypoint)

    trajectory[0] = list(poses_deg[0])
    trajectory[-1] = list(poses_deg[-1])
    return trajectory
