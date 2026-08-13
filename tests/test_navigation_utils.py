"""Unit tests for the pure navigation geometry helpers: no system, no hardware."""
import numpy as np
import pytest
from rosys.geometry import Point, Pose, PoseStep, Spline

from feldfreund_devkit.navigation import Reach, tool_reach
from feldfreund_devkit.navigation.utils import _solve_tool_t

TOOL_OFFSET = 0.09


def _straight(length: float = 2.0) -> Spline:
    return Spline.from_poses(Pose(x=0.0, y=0.0, yaw=0.0), Pose(x=length, y=0.0, yaw=0.0))


def _curved() -> Spline:
    """An arc far tighter than any crop row, to expose the curvature term."""
    return Spline.from_poses(Pose(x=0.0, y=0.0, yaw=0.0), Pose(x=1.0, y=1.0, yaw=np.pi / 2))


def _pose_with_tool_at(spline: Spline, target: Point) -> Pose:
    t, _ = _solve_tool_t(spline, target, TOOL_OFFSET, -0.2, 1.2)
    return spline.pose(t)


@pytest.mark.parametrize('lateral', (0.0, 0.1, -0.15))
def test_straight_path_puts_the_tool_a_tool_length_short_of_the_target(lateral: float) -> None:
    """On a straight path the lateral offset is irrelevant: only the forward coordinate counts."""
    pose = _pose_with_tool_at(_straight(), Point(x=1.0, y=lateral))

    assert pose.x == pytest.approx(1.0 - TOOL_OFFSET, abs=1e-6)
    assert pose.y == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize('lateral', (0.0, 0.15, -0.15))
def test_the_tool_lands_on_the_target_even_on_a_curve(lateral: float) -> None:
    spline = _curved()
    target = spline.pose(0.5).transform(Point(x=0.0, y=lateral))

    pose = _pose_with_tool_at(spline, target)

    assert pose.relative_point(target).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


def test_beats_projecting_and_stepping_back_on_a_curve() -> None:
    """Perpendicular foot plus a step back lands off the path, and the round trip back onto it
    loses centimetres on a curve with a laterally offset target.
    """
    spline = _curved()
    target = spline.pose(0.5).transform(Point(x=0.0, y=0.15))
    stepped = spline.pose(spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)) \
        + PoseStep(linear=-TOOL_OFFSET, angular=0, time=0)
    round_tripped = spline.pose(spline.closest_point(stepped.x, stepped.y, t_min=-0.2, t_max=1.2))

    exact = _pose_with_tool_at(spline, target)

    assert abs(round_tripped.relative_point(target).x - TOOL_OFFSET) > 0.01
    assert exact.relative_point(target).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


@pytest.mark.parametrize(('target_x', 'expected_t'), [(99.0, 1.2), (-99.0, -0.2)])
def test_a_target_off_either_end_clamps_to_the_bound(target_x: float, expected_t: float) -> None:
    """Callers get a usable parameter plus the flag that says it is not a real solution."""
    t, inside = _solve_tool_t(_straight(), Point(x=target_x, y=0.0), TOOL_OFFSET, -0.2, 1.2)

    assert not inside
    assert t == pytest.approx(expected_t, abs=1e-6)


def test_the_tool_reaches_a_target_on_the_spline() -> None:
    spline = _straight()
    reach = tool_reach(spline, Point(x=1.0, y=0.0), TOOL_OFFSET)

    assert reach.where is Reach.ON
    assert spline.pose(reach.t).relative_point(Point(x=1.0, y=0.0)).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


@pytest.mark.parametrize(('target_x', 'expected'), [(2.5, Reach.BEYOND), (-1.0, Reach.BEHIND)])
def test_a_target_off_either_end_is_told_apart(target_x: float, expected: Reach) -> None:
    """The two call for opposite reactions: wait for the segment that contains it, or give up."""
    assert tool_reach(_straight(), Point(x=target_x, y=0.0), TOOL_OFFSET).where is expected


def test_a_target_already_at_the_tool_is_reached_where_the_robot_stands() -> None:
    """The robot is already there: the stop is at t=0, not out of range."""
    spline = _straight()
    target = spline.pose(0.0).transform(Point(x=TOOL_OFFSET, y=0.0))

    reach = tool_reach(spline, target, TOOL_OFFSET)

    assert reach.where is Reach.ON
    assert reach.t == pytest.approx(0.0, abs=1e-6)


def test_a_target_just_behind_the_tool_is_reached_within_the_tolerance() -> None:
    spline = _straight()
    target = spline.pose(0.0).transform(Point(x=TOOL_OFFSET - 0.005, y=0.0))

    assert tool_reach(spline, target, TOOL_OFFSET).where is Reach.BEHIND
    assert tool_reach(spline, target, TOOL_OFFSET, tolerance=0.01).where is Reach.ON
