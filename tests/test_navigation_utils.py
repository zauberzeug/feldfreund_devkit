"""Unit tests for the pure navigation geometry helpers: no system, no hardware."""
import numpy as np
import pytest
from rosys.geometry import Point, Pose, PoseStep, Spline

from feldfreund_devkit.navigation import pose_with_tool_at, tool_t

TOOL_OFFSET = 0.09


def _straight(length: float = 2.0) -> Spline:
    return Spline.from_poses(Pose(x=0.0, y=0.0, yaw=0.0), Pose(x=length, y=0.0, yaw=0.0))


def _curved() -> Spline:
    """A quarter-circle-ish arc, far tighter than any crop row, to expose the curvature term."""
    return Spline.from_poses(Pose(x=0.0, y=0.0, yaw=0.0), Pose(x=1.0, y=1.0, yaw=np.pi / 2))


@pytest.mark.parametrize('lateral', (0.0, 0.1, -0.15))
def test_straight_path_puts_the_tool_a_tool_length_short_of_the_target(lateral: float) -> None:
    """On a straight path the lateral offset is irrelevant: only the forward coordinate counts."""
    pose = pose_with_tool_at(_straight(), Point(x=1.0, y=lateral), TOOL_OFFSET)

    assert pose.x == pytest.approx(1.0 - TOOL_OFFSET, abs=1e-6)
    assert pose.y == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize('lateral', (0.0, 0.15, -0.15))
def test_the_tool_lands_on_the_target_even_on_a_curve(lateral: float) -> None:
    """The defining property: the target sits exactly one tool length ahead in the robot's frame."""
    spline = _curved()
    target = spline.pose(0.5).transform(Point(x=0.0, y=lateral))

    pose = pose_with_tool_at(spline, target, TOOL_OFFSET)

    assert pose.relative_point(target).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


def test_beats_projecting_and_stepping_back_on_a_curve() -> None:
    """The previous reduction was exact as a pose, but its callers projected it back onto the path.

    Perpendicular foot plus a yaw-preserving step back does put the target a tool length ahead --
    of a pose that is off the path. Driving to it means projecting that pose back onto the spline,
    and on a curve with a laterally offset weed that round trip loses centimetres.
    """
    spline = _curved()
    target = spline.pose(0.5).transform(Point(x=0.0, y=0.15))
    stepped = spline.pose(spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)) \
        + PoseStep(linear=-TOOL_OFFSET, angular=0, time=0)
    round_tripped = spline.pose(spline.closest_point(stepped.x, stepped.y, t_min=-0.2, t_max=1.2))

    exact = pose_with_tool_at(spline, target, TOOL_OFFSET)

    assert abs(round_tripped.relative_point(target).x - TOOL_OFFSET) > 0.01
    assert exact.relative_point(target).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


def test_target_beyond_the_end_clamps_to_the_upper_bound() -> None:
    spline = _straight()
    assert pose_with_tool_at(spline, Point(x=99.0, y=0.0), TOOL_OFFSET, t_max=1.2).x \
        == pytest.approx(spline.pose(1.2).x, abs=1e-6)


def test_target_behind_the_start_clamps_to_the_lower_bound() -> None:
    spline = _straight()
    assert pose_with_tool_at(spline, Point(x=-99.0, y=0.0), TOOL_OFFSET, t_min=-0.2).x \
        == pytest.approx(spline.pose(-0.2).x, abs=1e-6)


def test_tool_t_finds_the_parameter_within_the_spline() -> None:
    spline = _straight()
    t = tool_t(spline, Point(x=1.0, y=0.0), TOOL_OFFSET)

    assert t is not None
    assert spline.pose(t).relative_point(Point(x=1.0, y=0.0)).x == pytest.approx(TOOL_OFFSET, abs=1e-6)


@pytest.mark.parametrize('target_x', (2.5, -1.0))
def test_tool_t_refuses_a_target_the_spline_does_not_reach(target_x: float) -> None:
    """Extrapolating would answer for a path the robot is not going to drive.

    ``Spline.pose`` happily evaluates outside ``[0, 1]``, and does so non-linearly, so a clamped or
    extrapolated answer looks plausible while being metres wrong.
    """
    assert tool_t(_straight(), Point(x=target_x, y=0.0), TOOL_OFFSET) is None


def test_tool_t_accepts_a_target_already_at_the_tool() -> None:
    """The robot is already there: the stop is at t=0, not out of range.

    A tool that works where it stands -- stop-and-go, or a weed reached while decelerating -- would
    otherwise have every one of its stops silently discarded.
    """
    spline = _straight()
    target = spline.pose(0.0).transform(Point(x=TOOL_OFFSET, y=0.0))

    t = tool_t(spline, target, TOOL_OFFSET)

    assert t is not None
    assert t == pytest.approx(0.0, abs=1e-6)
