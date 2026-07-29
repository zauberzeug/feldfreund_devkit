"""Unit tests for speed-cap composition: no hardware, only the arithmetic."""
import pytest
from rosys.geometry import Pose

from feldfreund_devkit.navigation import DriveSegment, PathDriver


def _path_driver(ambient: float = 0.13) -> PathDriver:
    return PathDriver(driver=None, speed_limit=lambda: ambient)  # type: ignore[arg-type]


def _segment(speed_limit: float | None = None) -> DriveSegment:
    return DriveSegment.from_poses(Pose(), Pose(x=1.0), speed_limit=speed_limit)


@pytest.mark.parametrize(('segment_limit', 'expected'), [(None, 0.13), (0.05, 0.05), (0.0, 0.0), (0.3, 0.13)])
def test_a_segment_can_only_slow_the_robot_down(segment_limit: float | None, expected: float) -> None:
    assert _path_driver().speed_limit(_segment(segment_limit)) == expected


def test_a_scoped_cap_applies_only_inside_its_scope() -> None:
    path_driver = _path_driver()
    segment = _segment()

    with path_driver.limit(0.04):
        assert path_driver.speed_limit(segment) == 0.04

    assert path_driver.speed_limit(segment) == 0.13


def test_the_slowest_of_everything_asked_for_wins() -> None:
    """Caps compose: the segment, the user and every scope get a veto, none can speed things up."""
    path_driver = _path_driver()

    with path_driver.limit(0.08), path_driver.limit(0.02), path_driver.limit(0.5):
        assert path_driver.speed_limit(_segment(0.06)) == 0.02
