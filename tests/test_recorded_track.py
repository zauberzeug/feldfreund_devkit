import math

import pytest
from conftest import GEO_REFERENCE
from rosys.geometry import GeoPose, GeoReference, Pose
from rosys.hardware.gnss import GpsQuality
from rosys.helpers import angle
from rosys.testing import forward

from feldfreund_devkit.navigation import (
    DriveSegment,
    GnssRequirement,
    RecordedTrack,
    RecordedTrackNavigation,
    RecordedWaypoint,
    generate_three_point_turn,
)

_LAT_DEG = math.degrees(GEO_REFERENCE.origin.lat)
_LON_DEG = math.degrees(GEO_REFERENCE.origin.lon)


def _make_waypoint(*, lat_offset: float = 0.0, approach_reverse: bool = False,
                   use_implement: bool = False, stop_at_waypoint: bool = False) -> RecordedWaypoint:
    return RecordedWaypoint(
        pose=GeoPose.from_degrees(_LAT_DEG + lat_offset, _LON_DEG, 0.0),
        approach_reverse=approach_reverse,
        use_implement=use_implement,
        stop_at_waypoint=stop_at_waypoint,
    )


def _make_track(waypoints: list[dict] | None = None) -> RecordedTrack:
    """Build a RecordedTrack with one waypoint per dict of keyword arguments."""
    track = RecordedTrack(name='test')
    for i, kwargs in enumerate(waypoints or [{}]):
        track._waypoints.append(_make_waypoint(lat_offset=i * 0.00001, **kwargs))
    return track


@pytest.fixture
def three_point_turn_track(devkit_system) -> RecordedTrack:
    """Three-point turn (0,0,0) to (0,1,180) plus a straight implement segment to (-2,1,180)."""
    assert GeoReference.current is not None
    track = RecordedTrack(name='three-point-turn')
    track._waypoints = [
        RecordedWaypoint(pose=GeoPose.from_pose(Pose(x=0.0, y=0.0, yaw=0.0))),
        RecordedWaypoint(pose=GeoPose.from_pose(Pose(x=1.500, y=1.500, yaw=math.radians(90.0)))),
        RecordedWaypoint(pose=GeoPose.from_pose(
            Pose(x=1.500, y=-0.500, yaw=math.radians(90.0))), approach_reverse=True),
        RecordedWaypoint(pose=GeoPose.from_pose(Pose(x=0.000, y=1.000, yaw=math.radians(180.0)))),
        RecordedWaypoint(pose=GeoPose.from_pose(Pose(x=-1.0, y=1.0, yaw=math.pi)), use_implement=True),
        RecordedWaypoint(pose=GeoPose.from_pose(Pose(x=-2.0, y=1.0, yaw=math.pi))),
    ]
    devkit_system.recorded_track_provider.recorded_tracks.append(track)
    devkit_system.recorded_track_provider.selected_track = track
    devkit_system.use_recorded_track_navigation()
    return track


# ---------------------------------------------------------------------------
# Pure-unit tests — RecordedWaypoint / RecordedTrack (no fixtures)
# ---------------------------------------------------------------------------

def test_roundtrip_serialization():
    track = _make_track([
        {'approach_reverse': False, 'use_implement': True, 'stop_at_waypoint': True},
        {'approach_reverse': True, 'use_implement': False, 'stop_at_waypoint': False},
        {},  # will use defaults
    ])
    restored = RecordedTrack.from_dict(track.to_dict())
    assert restored.waypoints[0].approach_reverse is False
    assert restored.waypoints[0].use_implement is True
    assert restored.waypoints[0].stop_at_waypoint is True
    assert restored.waypoints[1].approach_reverse is True
    assert restored.waypoints[1].use_implement is False
    assert restored.waypoints[1].stop_at_waypoint is False
    # check defaults
    assert restored.waypoints[2].approach_reverse is False
    assert restored.waypoints[2].use_implement is False
    assert restored.waypoints[2].stop_at_waypoint is False


def test_gnss_requirement_serialization():
    for requirement in GnssRequirement:
        track = _make_track([{}])
        track.gnss_requirement = requirement
        restored = RecordedTrack.from_dict(track.to_dict())
        assert restored.gnss_requirement == requirement


def test_gnss_requirement_defaults_to_rtk():
    track = _make_track([{}])
    data = track.to_dict()
    del data['gnss_requirement']
    restored = RecordedTrack.from_dict(data)
    assert restored.gnss_requirement == GnssRequirement.RTK


def test_meets_gnss_requirement():
    track = RecordedTrack(name='test')

    track.gnss_requirement = GnssRequirement.NONE
    assert track.meets_gnss_requirement(None) is True
    assert track.meets_gnss_requirement(GpsQuality.INVALID) is True
    assert track.meets_gnss_requirement(GpsQuality.GPS) is True

    track.gnss_requirement = GnssRequirement.GNSS
    assert track.meets_gnss_requirement(None) is False
    assert track.meets_gnss_requirement(GpsQuality.INVALID) is False
    assert track.meets_gnss_requirement(GpsQuality.GPS) is False
    assert track.meets_gnss_requirement(GpsQuality.DGPS) is True
    assert track.meets_gnss_requirement(GpsQuality.RTK_FIXED) is True
    assert track.meets_gnss_requirement(GpsQuality.RTK_FLOAT) is True

    track.gnss_requirement = GnssRequirement.RTK
    assert track.meets_gnss_requirement(None) is False
    assert track.meets_gnss_requirement(GpsQuality.DGPS) is False
    assert track.meets_gnss_requirement(GpsQuality.RTK_FLOAT) is False
    assert track.meets_gnss_requirement(GpsQuality.RTK_FIXED) is True


def test_reversed_three_point_turn_headings(three_point_turn_track: RecordedTrack):
    """All headings are rotated 180° when reversing, regardless of approach_reverse."""
    forward_yaws = [wp.pose.to_local().yaw for wp in three_point_turn_track.waypoints]
    reversed_waypoints = list(reversed(three_point_turn_track.waypoints))
    for i, wp in enumerate(reversed_waypoints):
        original_idx = len(three_point_turn_track.waypoints) - 1 - i
        reversed_yaw = wp.pose.to_local().yaw + math.pi
        assert reversed_yaw == pytest.approx(forward_yaws[original_idx] + math.pi, abs=1e-6)


# ---------------------------------------------------------------------------
# Integration tests — forward and reversed three-point-turn track with working segments before and after
# ---------------------------------------------------------------------------

async def test_three_point_turn_headings(devkit_system, three_point_turn_track: RecordedTrack):
    assert isinstance(devkit_system.current_navigation, RecordedTrackNavigation)
    test_poses = []

    def record_pose():
        pose = Pose(x=devkit_system.robot_locator.pose.point.x,
                    y=devkit_system.robot_locator.pose.point.y,
                    yaw=devkit_system.robot_locator.pose.yaw)
        test_poses.append(pose)
    devkit_system.automator.start()
    devkit_system.recorded_track_navigation.SEGMENT_COMPLETED.subscribe(record_pose)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert test_poses[0].x == pytest.approx(1.5, abs=0.05)
    assert test_poses[0].y == pytest.approx(1.5, abs=0.05)
    assert test_poses[0].yaw_deg == pytest.approx(90.0, abs=2.0)

    assert test_poses[1].x == pytest.approx(1.5, abs=0.05)
    assert test_poses[1].y == pytest.approx(-0.5, abs=0.05)
    assert test_poses[1].yaw_deg == pytest.approx(90.0, abs=2.0)

    assert test_poses[2].x == pytest.approx(0.0, abs=0.05)
    assert test_poses[2].y == pytest.approx(1.0, abs=0.05)
    assert test_poses[2].yaw_deg == pytest.approx(180.0, abs=2.0)


@pytest.mark.parametrize('reverse', (False, True))
async def test_approach_recorded_track(devkit_system, three_point_turn_track: RecordedTrack, reverse: bool):
    assert isinstance(devkit_system.current_navigation, RecordedTrackNavigation)
    devkit_system.recorded_track_navigation.reverse = reverse
    devkit_system.automator.start(devkit_system.recorded_track_navigation.approach_start())
    if reverse:
        expected_pose = three_point_turn_track.waypoints[-1].pose.to_local().rotate(math.pi)
        devkit_system.set_robot_pose(Pose(x=-3.0, y=1.0, yaw=math.pi))
    else:
        expected_pose = three_point_turn_track.waypoints[0].pose.to_local()
        devkit_system.set_robot_pose(Pose(x=-1.0, y=0.0, yaw=0.0))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)
    assert devkit_system.robot_locator.pose.point.x == pytest.approx(expected_pose.x, abs=0.1)
    assert devkit_system.robot_locator.pose.point.y == pytest.approx(expected_pose.y, abs=0.1)
    assert angle(expected_pose.yaw, devkit_system.robot_locator.pose.yaw) == pytest.approx(0, abs=math.radians(0.5))


@pytest.mark.parametrize('reverse', (False, True))
async def test_recorded_track_navigation(devkit_system, reverse: bool):
    """Robot drives a three-point-turn track with implement segments in both directions."""
    if reverse:
        start_pose = Pose(x=0.0, y=3.0, yaw=0.0)
        end_pose = Pose(x=1.0, y=0.0, yaw=math.pi)
        devkit_system.set_robot_pose(start_pose)
        segments = [
            DriveSegment.from_poses(Pose(x=1.0, y=3.0, yaw=0.0),
                                    Pose(x=2.0, y=3.0, yaw=0.0),
                                    use_implement=True),
            *generate_three_point_turn(Pose(x=2.0, y=3.0, yaw=0.0),
                                       Pose(x=2.0, y=1.0, yaw=math.pi)),
            DriveSegment.from_poses(Pose(x=2.0, y=1.0, yaw=math.pi),
                                    Pose(x=1.0, y=0.0, yaw=math.pi),
                                    use_implement=True,
                                    stop_at_end=True)
        ]

    else:
        start_pose = Pose(x=0.0, y=0.0, yaw=0.0)
        end_pose = Pose(x=1.0, y=3.0, yaw=math.pi)
        segments = [
            DriveSegment.from_poses(Pose(x=1.0, y=0.0, yaw=0.0),
                                    Pose(x=2.0, y=1.0, yaw=0.0),
                                    use_implement=True,
                                    stop_at_end=False),
            *generate_three_point_turn(Pose(x=2.0, y=1.0, yaw=0.0),
                                       Pose(x=2.0, y=3.0, yaw=math.pi)),
            DriveSegment.from_poses(Pose(x=2.0, y=3.0, yaw=math.pi),
                                    Pose(x=1.0, y=3.0, yaw=math.pi),
                                    use_implement=True,
                                    stop_at_end=True)
        ]

    waypoints = [
        RecordedWaypoint(pose=GeoPose(lat=0.9072805071005496, lon=0.12975200842318343, heading=0.0)),
        RecordedWaypoint(pose=GeoPose(lat=0.9072806640617644, lon=0.1297517535706485, heading=0.0),
                         use_implement=True),
        RecordedWaypoint(pose=GeoPose(lat=0.9072808995035275, lon=0.12975137129165418, heading=-math.pi / 2),
                         stop_at_waypoint=True),
        RecordedWaypoint(pose=GeoPose(lat=0.9072808995035905, lon=0.12975162614426589, heading=-math.pi / 2),
                         stop_at_waypoint=True, approach_reverse=True),
        RecordedWaypoint(pose=GeoPose(lat=0.9072806640616384, lon=0.1297512438655786, heading=-math.pi),
                         stop_at_waypoint=True),
        RecordedWaypoint(pose=GeoPose(lat=0.9072805071004079, lon=0.1297512438657321, heading=-math.pi),
                         use_implement=True),
    ]

    track = RecordedTrack(name='three-point-turn')
    track._waypoints = waypoints  # pylint: disable=protected-access
    devkit_system.recorded_track_provider.recorded_tracks.append(track)
    devkit_system.recorded_track_provider.selected_track = track
    devkit_system.use_recorded_track_navigation()
    assert isinstance(devkit_system.current_navigation, RecordedTrackNavigation)
    devkit_system.recorded_track_navigation.reverse = reverse

    assert devkit_system.robot_locator.pose.point.x == pytest.approx(start_pose.x, abs=0.1)
    assert devkit_system.robot_locator.pose.point.y == pytest.approx(start_pose.y, abs=0.1)
    assert devkit_system.robot_locator.pose.yaw == pytest.approx(start_pose.yaw, abs=0.1)
    devkit_system.automator.start()
    await forward(until=lambda: devkit_system.automator.is_running)
    assert len(devkit_system.recorded_track_navigation.path) == len(segments)

    for i, segment in enumerate(segments):
        path = devkit_system.recorded_track_navigation.path
        assert path[i].start.x == pytest.approx(segment.start.x, abs=0.1)
        assert path[i].start.y == pytest.approx(segment.start.y, abs=0.1)
        assert path[i].start.yaw == pytest.approx(segment.start.yaw, abs=0.1)
        assert path[i].end.x == pytest.approx(segment.end.x, abs=0.1)
        assert path[i].end.y == pytest.approx(segment.end.y, abs=0.1)
        assert path[i].end.yaw == pytest.approx(segment.end.yaw, abs=0.1)
        assert path[i].use_implement == segment.use_implement
        assert path[i].stop_at_end == segment.stop_at_end
        assert path[i].backward == segment.backward

    await forward(until=lambda: devkit_system.automator.is_stopped, timeout=120)
    assert devkit_system.robot_locator.pose.point.x == pytest.approx(end_pose.x, abs=0.1)
    assert devkit_system.robot_locator.pose.point.y == pytest.approx(end_pose.y, abs=0.1)
    assert angle(end_pose.yaw, devkit_system.robot_locator.pose.yaw) == pytest.approx(0, abs=0.1)
