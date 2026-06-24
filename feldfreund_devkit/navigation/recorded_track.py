import logging
from dataclasses import dataclass
from datetime import datetime
from enum import StrEnum
from typing import Any, Self
from uuid import uuid4

import rosys
from nicegui import Event
from rosys.geometry import GeoPose
from rosys.hardware.gnss import GpsQuality


class GnssRequirement(StrEnum):
    """Minimum GPS quality required when recording waypoints."""
    NONE = 'none'
    GNSS = 'gnss'
    RTK = 'rtk'

    def is_met_by(self, gps_quality: GpsQuality | None) -> bool:
        """Check whether the given GPS quality meets this requirement."""
        if self == GnssRequirement.NONE:
            return True
        if gps_quality is None:
            return False
        if self == GnssRequirement.GNSS:
            return gps_quality in {GpsQuality.DGPS, GpsQuality.PPS, GpsQuality.RTK_FIXED, GpsQuality.RTK_FLOAT}
        return gps_quality == GpsQuality.RTK_FIXED


@dataclass(slots=True, kw_only=True)
class RecordedWaypoint:
    """A waypoint with optional drive settings flags."""
    pose: GeoPose
    approach_reverse: bool = False
    use_implement: bool = False
    stop_at_waypoint: bool = False

    @property
    def degree_tuple(self):
        return self.pose.point.degree_tuple

    def to_dict(self) -> dict[str, Any]:
        return {
            'pose': self.pose.degree_tuple,
            'approach_reverse': self.approach_reverse,
            'use_implement': self.use_implement,
            'stop_at_waypoint': self.stop_at_waypoint,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'RecordedWaypoint':
        return cls(
            pose=GeoPose.from_degrees(*data['pose']),
            approach_reverse=data.get('approach_reverse', False),
            use_implement=data.get('use_implement', False),
            stop_at_waypoint=data.get('stop_at_waypoint', False),
        )


class RecordedTrack:
    """A track (basically a list of waypoints) that can be recorded and later navigated along."""

    def __init__(self, name: str | None = None,) -> None:
        self.id = str(uuid4())
        self.name = name if name is not None else datetime.now().strftime('%Y-%m-%d_%H-%M-%S.%f')
        self.gnss_requirement: GnssRequirement = GnssRequirement.RTK
        self._waypoints: list[RecordedWaypoint] = []

    def meets_gnss_requirement(self, gps_quality: GpsQuality | None) -> bool:
        """Check whether the given GPS quality meets this track's minimum requirement."""
        return self.gnss_requirement.is_met_by(gps_quality)

    def add_waypoint(self, waypoint: GeoPose, approach_reverse: bool = False) -> None:
        """Add a waypoint to the end of the list."""
        self._waypoints.append(RecordedWaypoint(pose=waypoint, approach_reverse=approach_reverse))

    def remove_waypoint(self, index: int) -> None:
        """Remove a waypoint at the specified index."""
        if not self._is_valid_index(index):
            raise IndexError(f'Waypoint index {index} is out of bounds (track has {len(self._waypoints)} waypoints)')
        self._waypoints.pop(index)

    def move_waypoint(self, from_index: int, to_index: int) -> None:
        """Move a waypoint from one position to another."""
        if not self._is_valid_index(from_index):
            raise IndexError(f'from_index {from_index} is out of bounds (track has {len(self._waypoints)} waypoints)')
        if not self._is_valid_index(to_index):
            raise IndexError(f'to_index {to_index} is out of bounds (track has {len(self._waypoints)} waypoints)')
        waypoint = self._waypoints.pop(from_index)
        self._waypoints.insert(to_index, waypoint)

    def _is_valid_index(self, index: int) -> bool:
        return 0 <= index < len(self._waypoints)

    def get_waypoint(self, index: int) -> RecordedWaypoint:
        """Get the waypoint at the given index, raising IndexError if out of bounds."""
        if not self._is_valid_index(index):
            raise IndexError(f'Waypoint index {index} is out of bounds (track has {len(self._waypoints)} waypoints)')
        return self._waypoints[index]

    def set_waypoint_approach_reverse(self, index: int, approach_reverse: bool) -> None:
        """Set whether to approach the waypoint at the given index in reverse."""
        self.get_waypoint(index).approach_reverse = approach_reverse

    def set_waypoint_use_implement(self, index: int, use_implement: bool) -> None:
        """Set whether to allow implement usage on the segment leading to this waypoint."""
        self.get_waypoint(index).use_implement = use_implement

    def set_waypoint_stop_at_waypoint(self, index: int, stop_at_waypoint: bool) -> None:
        """Set whether to stop at this waypoint."""
        self.get_waypoint(index).stop_at_waypoint = stop_at_waypoint

    def clear(self) -> None:
        """Remove all waypoints."""
        self._waypoints.clear()

    @property
    def first_waypoint(self) -> RecordedWaypoint | None:
        """Get the first waypoint."""
        return self._waypoints[0] if self._waypoints else None

    @property
    def last_waypoint(self) -> RecordedWaypoint | None:
        """Get the last waypoint."""
        return self._waypoints[-1] if self._waypoints else None

    @property
    def waypoints(self) -> list[RecordedWaypoint]:
        """Get the waypoints."""
        return self._waypoints

    def to_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'gnss_requirement': str(self.gnss_requirement),
            'waypoints': [wp.to_dict() for wp in self.waypoints],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        new_object = cls(data['name'])
        new_object.id = data['id']
        new_object.gnss_requirement = GnssRequirement(data.get('gnss_requirement', 'rtk'))
        new_object._waypoints = [RecordedWaypoint.from_dict(wp) for wp in data.get('waypoints', [])]
        return new_object


class RecordedTrackProvider(rosys.persistence.Persistable):
    """A provider of recorded waypoints."""

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('feldfreund_devkit.recorded_track_provider')
        self.recorded_tracks: list[RecordedTrack] = []
        self.selected_track: RecordedTrack | None = None

        """The recorded tracks have changed."""
        self.RECORDED_TRACKS_CHANGED: Event = Event()
        """ The selected track has changed. """
        self.RECORDED_TRACK_SELECTED: Event = Event()

    def select_track(self, track_id: str) -> None:
        """Select a track by ID."""
        self.selected_track = self.get_recorded_track(track_id)
        self.RECORDED_TRACK_SELECTED.emit()
        self.request_backup()

    def notify_track_modified(self) -> None:
        """Persist changes to an existing track (e.g. waypoint added/removed/moved, name changed)."""
        self.request_backup()

    def remove_recorded_track(self, track_id: str) -> None:
        """Remove a recorded track from the list."""
        recorded_track = self.get_recorded_track(track_id)
        if recorded_track is None:
            self.log.warning('Track with ID %s not found', track_id)
            return
        self.recorded_tracks.remove(recorded_track)
        if self.selected_track == recorded_track:
            self.selected_track = None
            self.RECORDED_TRACK_SELECTED.emit()
        self.RECORDED_TRACKS_CHANGED.emit()
        self.request_backup()

    def get_recorded_track(self, track_id: str) -> RecordedTrack | None:
        """Get the recorded track by ID."""
        track = next((track for track in self.recorded_tracks if track.id == track_id), None)
        if track is None:
            self.log.warning('Track with ID %s not found', track_id)
            return None
        return track

    def get_track_names_by_id(self) -> dict[str, str]:
        """Get the track names by their IDs."""
        return {track.id: track.name for track in self.recorded_tracks}

    def add_recorded_track(self, track: RecordedTrack) -> None:
        """Add a recorded track to the list."""
        self.recorded_tracks.append(track)
        self.RECORDED_TRACKS_CHANGED.emit()
        self.request_backup()

    def backup_to_dict(self) -> dict[str, Any]:
        """Backup the recorded tracks to a dictionary."""
        return {
            'recorded_tracks': [
                track.to_dict()
                for track in self.recorded_tracks
            ],
            'selected_track': self.selected_track.id if self.selected_track else None,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        """Restore the recorded tracks from a dictionary."""
        self.recorded_tracks.clear()
        for track_data in data.get('recorded_tracks', []):
            track = RecordedTrack.from_dict(track_data)
            self.recorded_tracks.append(track)
        selected_track_id = data.get('selected_track', None)
        if selected_track_id is not None:
            self.select_track(selected_track_id)
