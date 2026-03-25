import math

import pytest
from rosys.geometry import Pose

from feldfreund_devkit.config import RobotFootprint


@pytest.fixture
def footprint() -> RobotFootprint:
    return RobotFootprint(front=0.35, rear=0.35, left=0.30, right=0.30)


def test_corners_at_identity_pose(footprint: RobotFootprint) -> None:
    corners = footprint.corners_at_pose(Pose(x=0.0, y=0.0, yaw=0.0))
    fl, fr, rr, rl = corners
    assert fl.x == pytest.approx(0.35) and fl.y == pytest.approx(0.30)
    assert fr.x == pytest.approx(0.35) and fr.y == pytest.approx(-0.30)
    assert rr.x == pytest.approx(-0.35) and rr.y == pytest.approx(-0.30)
    assert rl.x == pytest.approx(-0.35) and rl.y == pytest.approx(0.30)


def test_corners_at_rotated_pose(footprint: RobotFootprint) -> None:
    # yaw=pi/2: robot faces +y in world frame
    # transform: x' = -point.y, y' = point.x
    corners = footprint.corners_at_pose(Pose(x=0.0, y=0.0, yaw=math.pi / 2))
    fl, fr, rr, rl = corners
    assert fl.x == pytest.approx(-0.30) and fl.y == pytest.approx(0.35)
    assert fr.x == pytest.approx(0.30) and fr.y == pytest.approx(0.35)
    assert rr.x == pytest.approx(0.30) and rr.y == pytest.approx(-0.35)
    assert rl.x == pytest.approx(-0.30) and rl.y == pytest.approx(-0.35)


def test_corners_at_offset_pose(footprint: RobotFootprint) -> None:
    # Translated but not rotated: corners shift by pose offset
    corners = footprint.corners_at_pose(Pose(x=1.0, y=2.0, yaw=0.0))
    fl, fr, rr, rl = corners
    assert fl.x == pytest.approx(1.35) and fl.y == pytest.approx(2.30)
    assert fr.x == pytest.approx(1.35) and fr.y == pytest.approx(1.70)
    assert rr.x == pytest.approx(0.65) and rr.y == pytest.approx(1.70)
    assert rl.x == pytest.approx(0.65) and rl.y == pytest.approx(2.30)


def test_corners_at_rotated_and_offset_pose(footprint: RobotFootprint) -> None:
    # Pose at (1.0, 2.0) facing +y (yaw=pi/2)
    # transform: x' = 1.0 - point.y, y' = 2.0 + point.x
    corners = footprint.corners_at_pose(Pose(x=1.0, y=2.0, yaw=math.pi / 2))
    fl, fr, rr, rl = corners
    assert fl.x == pytest.approx(0.70) and fl.y == pytest.approx(2.35)
    assert fr.x == pytest.approx(1.30) and fr.y == pytest.approx(2.35)
    assert rr.x == pytest.approx(1.30) and rr.y == pytest.approx(1.65)
    assert rl.x == pytest.approx(0.70) and rl.y == pytest.approx(1.65)
