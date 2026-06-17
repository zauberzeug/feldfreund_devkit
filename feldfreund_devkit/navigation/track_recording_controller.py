from __future__ import annotations

import logging
from typing import Protocol, runtime_checkable

import rosys
from nicegui import Event
from rosys.automation import AppButton
from rosys.driving.driver import PoseProvider
from rosys.geometry import GeoPose
from rosys.hardware import Gnss

from .recorded_track import RecordedTrack, RecordedTrackProvider

log = logging.getLogger('feldfreund_devkit.track_recording_controller')


@runtime_checkable
class AppButtonControls(Protocol):
    """Minimal interface for the bluetooth-app button bar the controller needs."""

    async def add_button(self, key: str, button: AppButton) -> None: ...

    async def remove_button(self, key: str) -> None: ...


class TrackRecordingController:
    """Server-side recording mode that survives client disconnects.

    Owns the bluetooth-app waypoint button so the operator can keep adding
    waypoints even if the browser/dialog is gone.

    The app button bar (``app_controls``) is bound late: it is created by the
    application after this controller and only on real hardware, so it stays
    ``None`` until assigned and the button registration is skipped while absent.
    """

    APP_BUTTON_KEY = 'record_waypoint'

    def __init__(self, recorded_track_provider: RecordedTrackProvider, *,
                 pose_provider: PoseProvider, gnss: Gnss | None = None) -> None:
        self.recorded_track_provider = recorded_track_provider
        self.pose_provider = pose_provider
        self.gnss = gnss
        self.app_controls: AppButtonControls | None = None
        self._active_track: RecordedTrack | None = None
        self._track_was_new: bool = False
        self._started_at: float | None = None

        self.RECORDING_STARTED: Event = Event()
        self.RECORDING_STOPPED: Event = Event()
        self.WAYPOINT_ADDED: Event = Event()

    @property
    def active_track(self) -> RecordedTrack | None:
        return self._active_track

    @property
    def is_recording(self) -> bool:
        return self._active_track is not None

    @property
    def started_at(self) -> float | None:
        return self._started_at

    @property
    def elapsed_seconds(self) -> float:
        return 0.0 if self._started_at is None else max(0.0, rosys.time() - self._started_at)

    async def start_recording(self, track: RecordedTrack) -> bool:
        """Start a recording session. Returns False if a session is already active.

        Persists the track immediately (so it survives a server crash) without
        emitting `RECORDED_TRACKS_CHANGED` — that event refreshes the settings UI
        which would tear down a dialog opened from there. Stop emits it instead.
        """
        if self._active_track is not None:
            rosys.notify('A recording is already in progress', 'warning')
            return False
        provider = self.recorded_track_provider
        if track not in provider.recorded_tracks:
            provider.recorded_tracks.append(track)
            self._track_was_new = True
        else:
            self._track_was_new = False
        provider.request_backup()
        self._active_track = track
        self._started_at = rosys.time()
        await self._register_app_button()
        self.RECORDING_STARTED.emit()
        log.info('Recording started for track %s (%s)', track.name, track.id)
        return True

    async def stop_recording(self) -> None:
        """Stop the active recording. Idempotent."""
        if self._active_track is None:
            return
        track = self._active_track
        was_new = self._track_was_new
        self._active_track = None
        self._track_was_new = False
        self._started_at = None
        await self._unregister_app_button()
        # Close any open dialog first; only then refresh the settings panel
        # (settings refresh would otherwise tear down the dialog mid-close).
        self.RECORDING_STOPPED.emit()
        provider = self.recorded_track_provider
        provider.RECORDED_TRACKS_CHANGED.emit()
        if was_new:
            provider.select_track(track.id)
        log.info('Recording stopped for track %s (%s)', track.name, track.id)

    async def add_waypoint_at_current_pose(self) -> None:
        """Append the robot's current pose as a waypoint to the active track.

        Returns without adding if not recording or if the GNSS quality is
        insufficient. The GNSS case notifies the operator (forwarded to both
        browser clients and the bluetooth app via `rosys.NEW_NOTIFICATION`).
        """
        if self._active_track is None:
            log.warning('Waypoint not added: no active recording')
            return
        if not self._meets_gnss_requirement(self._active_track):
            rosys.notify('Waypoint not added: GNSS quality insufficient', 'warning')
            return
        geo_pose = GeoPose.from_pose(self.pose_provider.pose)
        self._active_track.add_waypoint(geo_pose)
        self.recorded_track_provider.notify_track_modified()
        self.WAYPOINT_ADDED.emit()
        log.info('Waypoint %d added at %s', len(self._active_track.waypoints), geo_pose)

    def _meets_gnss_requirement(self, track: RecordedTrack) -> bool:
        if self.gnss is None:
            return True
        measurement = self.gnss.last_measurement
        gps_quality = measurement.gps_quality if measurement else None
        return track.meets_gnss_requirement(gps_quality)

    async def _register_app_button(self) -> None:
        if self.app_controls is None:
            return
        await self.app_controls.add_button(
            self.APP_BUTTON_KEY,
            AppButton('add_location', released=self.add_waypoint_at_current_pose),  # type: ignore[arg-type]
        )

    async def _unregister_app_button(self) -> None:
        if self.app_controls is None:
            return
        await self.app_controls.remove_button(self.APP_BUTTON_KEY)
