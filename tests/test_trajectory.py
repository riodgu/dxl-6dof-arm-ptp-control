"""Tests for quintic interpolation and joint trajectory generation."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dxl_arm.trajectory import (
    generate_continuous_joint_trajectory,
    generate_joint_trajectory,
    quintic_interpolation_ratio,
)


def test_quintic_ratio_at_zero():
    assert quintic_interpolation_ratio(0.0) == 0.0


def test_quintic_ratio_at_one():
    assert quintic_interpolation_ratio(1.0) == 1.0


def test_quintic_ratio_midpoint_is_half():
    assert abs(quintic_interpolation_ratio(0.5) - 0.5) < 1e-9


def test_quintic_ratio_monotonic():
    ratios = [quintic_interpolation_ratio(r / 10) for r in range(11)]
    assert ratios == sorted(ratios)


def test_trajectory_first_point_equals_start():
    start = [0, 0, 0, 0, 0]
    goal = [10, 20, -10, 5, 0]
    traj = generate_joint_trajectory(start, goal, move_time=1.0, hz=100.0)
    assert traj[0] == start


def test_trajectory_last_point_equals_goal():
    start = [0, 0, 0, 0, 0]
    goal = [10, 20, -10, 5, 0]
    traj = generate_joint_trajectory(start, goal, move_time=1.0, hz=100.0)
    assert traj[-1] == goal


def test_trajectory_length_matches_expected():
    move_time = 2.0
    hz = 50.0
    start = [0, 0]
    goal = [10, -10]
    traj = generate_joint_trajectory(start, goal, move_time=move_time, hz=hz)
    expected_len = int(round(move_time * hz)) + 1
    assert len(traj) == expected_len


def test_trajectory_each_waypoint_has_correct_joint_count():
    start = [0, 0, 0]
    goal = [1, 2, 3]
    traj = generate_joint_trajectory(start, goal, move_time=0.5, hz=20.0)
    for waypoint in traj:
        assert len(waypoint) == 3


def test_trajectory_linear_method_reaches_goal():
    start = [0.0]
    goal = [100.0]
    traj = generate_joint_trajectory(start, goal, move_time=1.0, hz=10.0, method="linear")
    assert traj[-1] == goal


def test_continuous_trajectory_passes_through_internal_pose_without_duplicate():
    traj = generate_continuous_joint_trajectory(
        [[0.0], [10.0], [20.0]], [1.0, 1.0], hz=10.0
    )

    assert traj[0] == [0.0]
    assert traj[10] == [10.0]
    assert traj[-1] == [20.0]
    assert len(traj) == 21


def test_continuous_trajectory_does_not_overshoot_at_direction_change():
    traj = generate_continuous_joint_trajectory(
        [[0.0], [10.0], [5.0]], [1.0, 1.0], hz=20.0
    )

    assert all(0.0 <= waypoint[0] <= 10.0 for waypoint in traj)
