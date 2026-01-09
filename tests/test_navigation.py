import numpy as np
import pytest
import rosys
from rosys.geometry import Point, Pose
from rosys.helpers import angle
from rosys.testing import assert_point, forward

from feldfreund_devkit.hardware.tracks import TracksSimulation
from feldfreund_devkit.navigation import DriveSegment, StraightLineNavigation


@pytest.mark.parametrize('distance', (0.005, 0.01, 0.05, 0.1, 0.5, 1.0))
async def test_stopping_at_different_distances(devkit_system, distance: float):
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)
    devkit_system.current_navigation.length = distance
    devkit_system.current_navigation.linear_speed_limit = 0.13
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    assert devkit_system.current_navigation.current_segment is not None
    assert devkit_system.current_navigation.current_segment.spline.estimated_length() == distance
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert devkit_system.robot_locator.pose.point.x == pytest.approx(distance, abs=0.0015)


@pytest.mark.parametrize('heading_degrees', (-180, -90, -45, 0, 45, 90, 180, 360))
async def test_straight_line_different_headings(devkit_system, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    current_pose = devkit_system.robot_locator.pose
    devkit_system.set_robot_pose(Pose(x=current_pose.x, y=current_pose.y, yaw=heading))
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    current_segment = devkit_system.current_navigation.current_segment
    assert current_segment is not None
    direction = current_segment.spline.start.direction(current_segment.spline.end)
    assert angle(direction, heading) == pytest.approx(0, abs=0.1)


@pytest.mark.parametrize('distance', (0.005, 0.01, 0.05, 0.1, 0.5, 1.0))
async def test_deceleration_different_distances(devkit_system_with_acceleration, distance: float):
    assert isinstance(devkit_system_with_acceleration.feldfreund.wheels, TracksSimulation)
    assert isinstance(devkit_system_with_acceleration.current_navigation, StraightLineNavigation)
    devkit_system_with_acceleration.current_navigation.length = distance
    devkit_system_with_acceleration.current_navigation.linear_speed_limit = 0.13
    devkit_system_with_acceleration.automator.start()
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_running)
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_stopped)
    assert devkit_system_with_acceleration.robot_locator.pose.point.x == pytest.approx(distance, abs=0.0015)


@pytest.mark.parametrize(('linear_speed_limit', 'tolerance'), [
    (0.1, 0.001),
    (0.13, 0.001),
    (0.2, 0.002),
    (0.3, 0.0025),
    (0.4, 0.005),
])
async def test_deceleration_different_speeds(devkit_system_with_acceleration, linear_speed_limit: float, tolerance: float):
    assert isinstance(devkit_system_with_acceleration.feldfreund.wheels, TracksSimulation)
    assert isinstance(devkit_system_with_acceleration.current_navigation, StraightLineNavigation)
    devkit_system_with_acceleration.current_navigation.length = 0.005
    devkit_system_with_acceleration.current_navigation.linear_speed_limit = linear_speed_limit
    devkit_system_with_acceleration.automator.start()
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_running)
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_stopped)
    assert devkit_system_with_acceleration.robot_locator.pose.point.x == pytest.approx(0.005, abs=tolerance)


async def test_slippage(devkit_system):
    assert isinstance(devkit_system.feldfreund.wheels, rosys.hardware.WheelsSimulation)
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)
    devkit_system.current_navigation.length = 2.0
    devkit_system.feldfreund.wheels.slip_factor_right = 0.04
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert_point(devkit_system.robot_locator.pose.point, Point(x=2.0, y=0))


@pytest.mark.parametrize('start_offset', (0.5, 0.0, -0.25, -0.5, -0.75, -0.99))
async def test_start_inbetween_waypoints(devkit_system, start_offset: float):
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)
    # generate path which expands left and right from current pose
    start = devkit_system.robot_locator.pose.transform_pose(Pose(x=start_offset, y=0.0, yaw=0.0))
    end = start.transform_pose(Pose(x=1.0, y=0.0, yaw=0.0))
    devkit_system.current_navigation.generate_path = lambda: [  # type: ignore[assignment]
        DriveSegment.from_poses(start, end)]
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    assert devkit_system.current_navigation.current_segment is not None
    assert devkit_system.current_navigation.current_segment.end.x == pytest.approx(end.x, abs=0.1)
    assert devkit_system.current_navigation.current_segment.end.y == pytest.approx(end.y, abs=0.1)
    assert devkit_system.current_navigation.current_segment.end.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_start_on_end(devkit_system):
    segment_started = False

    def handle_segment_started(_: DriveSegment):
        nonlocal segment_started
        segment_started = True
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)
    # set start of path 1m before current pose
    start = devkit_system.robot_locator.pose.transform_pose(Pose(x=-1, y=0.0, yaw=0.0))
    end = devkit_system.robot_locator.pose
    devkit_system.current_navigation.generate_path = lambda: [  # type: ignore[assignment]
        DriveSegment.from_poses(start, end)]
    devkit_system.current_navigation.SEGMENT_STARTED.subscribe(handle_segment_started)
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    assert segment_started
    assert devkit_system.robot_locator.pose.x == pytest.approx(end.x, abs=0.1)
    assert devkit_system.robot_locator.pose.y == pytest.approx(end.y, abs=0.1)
    assert devkit_system.robot_locator.pose.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_skip_first_segment(devkit_system):
    pose1 = Pose(x=-1, y=1, yaw=-np.pi/2)
    pose2 = Pose(x=0, y=0.0, yaw=0.0)
    pose3 = Pose(x=1.0, y=1.0, yaw=np.pi/2)
    pose4 = Pose(x=0, y=2.0, yaw=np.pi)
    assert isinstance(devkit_system.current_navigation, StraightLineNavigation)

    def generate_path():
        path = [
            DriveSegment.from_poses(pose1, pose2, stop_at_end=False),
            DriveSegment.from_poses(pose2, pose3, stop_at_end=False),
            DriveSegment.from_poses(pose3, pose4, stop_at_end=False),
            DriveSegment.from_poses(pose4, pose1),
        ]
        assert devkit_system.current_navigation is not None
        path = devkit_system.current_navigation._remove_segments_behind_robot(path)  # pylint: disable=protected-access
        return path
    devkit_system.current_navigation.generate_path = generate_path  # type: ignore[assignment]
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.current_navigation is not None and devkit_system.current_navigation.current_segment is not None)
    assert devkit_system.current_navigation.current_segment is not None
    assert len(devkit_system.current_navigation.path) == 3
    assert devkit_system.current_navigation.current_segment.end.x == pytest.approx(pose3.x, abs=0.1)
    assert devkit_system.current_navigation.current_segment.end.y == pytest.approx(pose3.y, abs=0.1)
    assert devkit_system.current_navigation.current_segment.end.yaw_deg == pytest.approx(pose3.yaw_deg, abs=0.1)
