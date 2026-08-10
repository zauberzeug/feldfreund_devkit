import numpy as np
import pytest
import rosys
from rosys.geometry import Point, Pose
from rosys.helpers import angle
from rosys.testing import assert_point, forward

from feldfreund_devkit.hardware.tracks import TracksSimulation
from feldfreund_devkit.navigation import DriveSegment, skip_completed_segments


@pytest.mark.parametrize('distance', (0.005, 0.01, 0.05, 0.1, 0.5, 1.0))
async def test_stopping_at_different_distances(devkit_system, distance: float):
    devkit_system.straight_line_navigation.length = distance
    assert devkit_system.straight_line_navigation.generate_path(0.13)[0].spline.estimated_length() == distance
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert devkit_system.robot_locator.pose.point.x == pytest.approx(distance, abs=0.0015)


@pytest.mark.parametrize('heading_degrees', (-180, -90, -45, 0, 45, 90, 180, 360))
async def test_straight_line_different_headings(devkit_system, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    current_pose = devkit_system.robot_locator.pose
    devkit_system.set_robot_pose(Pose(x=current_pose.x, y=current_pose.y, yaw=heading))
    segment = devkit_system.straight_line_navigation.generate_path(0.13)[0]
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    direction = segment.spline.start.direction(segment.spline.end)
    assert angle(direction, heading) == pytest.approx(0, abs=0.1)


async def test_straight_line_backward(devkit_system):
    devkit_system.straight_line_navigation.length = 1.0
    devkit_system.straight_line_navigation.backward = True
    segment = devkit_system.straight_line_navigation.generate_path(0.13)[0]
    assert segment.backward is True
    assert segment.use_implement is False
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert devkit_system.robot_locator.pose.point.x == pytest.approx(-1.0, abs=0.0015)
    assert devkit_system.robot_locator.pose.point.y == pytest.approx(0.0, abs=0.0015)


@pytest.mark.parametrize('distance', (0.005, 0.01, 0.05, 0.1, 0.5, 1.0))
async def test_deceleration_different_distances(devkit_system_with_acceleration, distance: float):
    assert isinstance(devkit_system_with_acceleration.feldfreund.wheels, TracksSimulation)
    devkit_system_with_acceleration.straight_line_navigation.length = distance
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
    devkit_system_with_acceleration.straight_line_navigation.length = 0.005
    devkit_system_with_acceleration.straight_line_navigation.linear_speed_limit = linear_speed_limit
    devkit_system_with_acceleration.automator.start()
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_running)
    await forward(until=lambda: devkit_system_with_acceleration.automator.is_stopped)
    assert devkit_system_with_acceleration.robot_locator.pose.point.x == pytest.approx(0.005, abs=tolerance)


async def test_slippage(devkit_system):
    assert isinstance(devkit_system.feldfreund.wheels, rosys.hardware.WheelsSimulation)
    devkit_system.straight_line_navigation.length = 2.0
    devkit_system.feldfreund.wheels.slip_factor_right = 0.04
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert_point(devkit_system.robot_locator.pose.point, Point(x=2.0, y=0))


@pytest.mark.parametrize('start_offset', (0.5, 0.0, -0.25, -0.5, -0.75, -0.99))
async def test_start_inbetween_waypoints(devkit_system, start_offset: float):
    # a route which expands left and right from the current pose
    start = devkit_system.robot_locator.pose.transform_pose(Pose(x=start_offset, y=0.0, yaw=0.0))
    end = start.transform_pose(Pose(x=1.0, y=0.0, yaw=0.0))
    devkit_system.straight_line_navigation.generate_path = lambda speed_limit: [  # type: ignore[assignment]
        DriveSegment.from_poses(start, end, speed_limit=0.13)]
    driven: list[DriveSegment] = []
    devkit_system.path_driver.SEGMENT_STARTED.subscribe(driven.append)
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: driven, timeout=10)
    assert driven[0].end.x == pytest.approx(end.x, abs=0.1)
    assert driven[0].end.y == pytest.approx(end.y, abs=0.1)
    assert driven[0].end.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_start_on_end(devkit_system):
    segment_started = False

    def handle_segment_started(_: DriveSegment):
        nonlocal segment_started
        segment_started = True
    # set start of path 1m before current pose
    start = devkit_system.robot_locator.pose.transform_pose(Pose(x=-1, y=0.0, yaw=0.0))
    end = devkit_system.robot_locator.pose
    devkit_system.straight_line_navigation.generate_path = lambda speed_limit: [  # type: ignore[assignment]
        DriveSegment.from_poses(start, end, speed_limit=0.13)]
    devkit_system.path_driver.SEGMENT_STARTED.subscribe(handle_segment_started)
    devkit_system.automator.start()
    # NOTE: the robot already stands at the end, so the run is over too quickly to catch it running
    await forward(until=lambda: segment_started, timeout=10)
    assert devkit_system.robot_locator.pose.x == pytest.approx(end.x, abs=0.1)
    assert devkit_system.robot_locator.pose.y == pytest.approx(end.y, abs=0.1)
    assert devkit_system.robot_locator.pose.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_skip_first_segment(devkit_system):
    pose1 = Pose(x=-1, y=1, yaw=-np.pi / 2)
    pose2 = Pose(x=0, y=0.0, yaw=0.0)
    pose3 = Pose(x=1.0, y=1.0, yaw=np.pi / 2)
    pose4 = Pose(x=0, y=2.0, yaw=np.pi)

    def generate_path(speed_limit: float = 0.13):
        path = [
            DriveSegment.from_poses(pose1, pose2, stop_at_end=False, speed_limit=0.13),
            DriveSegment.from_poses(pose2, pose3, stop_at_end=False, speed_limit=0.13),
            DriveSegment.from_poses(pose3, pose4, stop_at_end=False, speed_limit=0.13),
            DriveSegment.from_poses(pose4, pose1, speed_limit=0.13),
        ]
        return skip_completed_segments(devkit_system.robot_locator.pose, path)
    devkit_system.straight_line_navigation.generate_path = generate_path  # type: ignore[assignment]
    planned = generate_path()
    driven: list[DriveSegment] = []
    devkit_system.path_driver.SEGMENT_STARTED.subscribe(driven.append)
    devkit_system.automator.start()
    await forward(until=lambda: driven, timeout=10)

    assert len(planned) == 3, 'the segment the robot stands at the end of is skipped'
    assert driven[0].end.x == pytest.approx(pose3.x, abs=0.1)
    assert driven[0].end.y == pytest.approx(pose3.y, abs=0.1)
    assert driven[0].end.yaw_deg == pytest.approx(pose3.yaw_deg, abs=0.1)


@pytest.mark.parametrize(('robot_x', 'robot_yaw_deg', 'expected_count', 'expected_start_x'), [
    (0.0, 0, 3, 0.0),  # at start of first segment, facing forward
    (0.5, 0, 3, 0.0),  # on first segment, facing forward
    (1.2, 0, 2, 1.0),  # on second segment, facing forward
    (2.5, 0, 1, 2.0),  # on third segment, facing forward
    (1.2, 30, 2, 1.0),  # on second segment, heading offset within tolerance
    (1.2, 60, 0, None),  # heading offset exceeds default 45° tolerance
    (1.2, 180, 0, None),  # facing backward, no segment is reachable
])
def test_skip_completed_segments(robot_x: float,
                                 robot_yaw_deg: float,
                                 expected_count: int,
                                 expected_start_x: float | None):
    pose0 = Pose(x=0.0, y=0.0, yaw=0.0)
    pose1 = Pose(x=1.0, y=0.0, yaw=0.0)
    pose2 = Pose(x=2.0, y=0.0, yaw=0.0)
    pose3 = Pose(x=3.0, y=0.0, yaw=0.0)
    path = [
        DriveSegment.from_poses(pose0, pose1, speed_limit=0.13),
        DriveSegment.from_poses(pose1, pose2, speed_limit=0.13),
        DriveSegment.from_poses(pose2, pose3, speed_limit=0.13),
    ]
    robot_pose = Pose(x=robot_x, y=0.0, yaw=np.deg2rad(robot_yaw_deg))
    result = skip_completed_segments(robot_pose, path)
    assert len(result) == expected_count
    if expected_count > 0:
        assert expected_start_x is not None
        assert result[0].start.x == pytest.approx(expected_start_x)
        assert result[0].start.y == pytest.approx(0.0)
        assert result[-1].end.x == pytest.approx(pose3.x)


def test_skip_completed_segments_picks_up_backward_segment():
    # robot drives backward from x=2 to x=1 (still facing +x), then forward from x=1 to x=3
    path = [
        DriveSegment.from_poses(Pose(x=2.0, y=0.0, yaw=0.0), Pose(x=1.0, y=0.0, yaw=0.0), backward=True, speed_limit=0.13),
        DriveSegment.from_poses(Pose(x=1.0, y=0.0, yaw=0.0), Pose(x=3.0, y=0.0, yaw=0.0), speed_limit=0.13),
    ]
    # robot mid-backward-segment, correctly facing +x → must accept the backward segment
    result = skip_completed_segments(Pose(x=1.5, y=0.0, yaw=0.0), path)
    assert len(result) == 2
    assert result[0].backward is True

    # same position but facing -x → wrong way for the backward leg AND for the forward continuation
    result = skip_completed_segments(Pose(x=1.5, y=0.0, yaw=np.pi), path)
    assert result == []


def test_skip_completed_segments_handles_segment_seam():
    # robot near the very end of segment 0 must continue from segment 1, not redrive segment 0
    path = [
        DriveSegment.from_poses(Pose(x=0.0, y=0.0, yaw=0.0), Pose(x=1.0, y=0.0, yaw=0.0), speed_limit=0.13),
        DriveSegment.from_poses(Pose(x=1.0, y=0.0, yaw=0.0), Pose(x=2.0, y=0.0, yaw=0.0), speed_limit=0.13),
        DriveSegment.from_poses(Pose(x=2.0, y=0.0, yaw=0.0), Pose(x=3.0, y=0.0, yaw=0.0), speed_limit=0.13),
    ]
    result = skip_completed_segments(Pose(x=0.999, y=0.0, yaw=0.0), path)
    assert len(result) == 2
    assert result[0].start.x == pytest.approx(1.0)
