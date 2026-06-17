from __future__ import annotations

import contextlib
import logging
import math
from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui
from rosys.automation import Automator
from rosys.geometry import Pose, Spline
from rosys.hardware import Gnss

from .drive_segment import DriveSegment
from .recorded_track import GnssRequirement, RecordedTrack, RecordedTrackProvider
from .track_recording_controller import TrackRecordingController
from .utils import skip_completed_segments
from .waypoint_navigation import WaypointNavigation

if TYPE_CHECKING:
    from ..interface.components.track_recorder_dialog import TrackRecorderDialog


class RecordedTrackNavigation(WaypointNavigation):
    """Drives along a previously recorded track of waypoints."""

    # Boundaries when starting the mid of track is allowed.
    RESUME_MAX_OFFSET: float = 2.0           # meters, perpendicular to path
    RESUME_MAX_HEADING: float = np.deg2rad(45)

    def __init__(self, *,
                 recorded_track_provider: RecordedTrackProvider,
                 track_recording_controller: TrackRecordingController,
                 gnss: Gnss | None = None,
                 automator: Automator | None = None,
                 **kwargs) -> None:
        super().__init__(**kwargs, name='Recorded Track Navigation')
        self.recorded_track_provider = recorded_track_provider
        self.track_recording_controller = track_recording_controller
        self.gnss = gnss
        self.automator = automator
        self.reverse: bool = False
        # Live waypoint count is patched directly into this label to avoid refreshing
        # the banner on every waypoint, which would disturb sibling elements.
        self._banner_count_label: ui.label | None = None
        self._settings_content_row: ui.column | None = None
        self._dialog_host: ui.element | None = None
        self._current_recorder: TrackRecorderDialog | None = None

        self.recorded_track_provider.RECORDED_TRACK_SELECTED.subscribe(self._settings_content.refresh)
        self.recorded_track_provider.RECORDED_TRACKS_CHANGED.subscribe(self._settings_content.refresh)
        self.track_recording_controller.RECORDING_STARTED.subscribe(self._recording_banner.refresh)
        self.track_recording_controller.RECORDING_STARTED.subscribe(lambda: self._set_settings_visible(False))
        self.track_recording_controller.RECORDING_STOPPED.subscribe(self._recording_banner.refresh)
        self.track_recording_controller.RECORDING_STOPPED.subscribe(lambda: self._set_settings_visible(True))
        self.track_recording_controller.WAYPOINT_ADDED.subscribe(self._update_banner_count)

    async def prepare(self) -> bool:
        if not await super().prepare():
            return False
        selected_track = self.recorded_track_provider.selected_track
        if selected_track is not None and selected_track.gnss_requirement != GnssRequirement.NONE:
            measurement = self.gnss.last_measurement if self.gnss else None
            if not selected_track.meets_gnss_requirement(measurement.gps_quality if measurement else None):
                rosys.notify('GNSS quality insufficient for this track', 'negative')
                self.log.warning('Navigation not started: GNSS requirement %s not met', selected_track.gnss_requirement)
                return False
        return True

    def generate_path(self) -> list[DriveSegment]:
        recorded_track = self.recorded_track_provider.selected_track
        if recorded_track is None:
            raise ValueError('No track selected')
        waypoints = recorded_track.waypoints
        path_segments: list[DriveSegment] = []
        for i in range(1, len(waypoints)):
            start_pose = waypoints[i - 1].pose.to_local()
            end_pose = waypoints[i].pose.to_local()
            backward = waypoints[i].approach_reverse
            spline = Spline.from_poses(start_pose, end_pose, backward=backward)
            is_last_segment = i == len(waypoints) - 1
            path_segments.append(DriveSegment(
                spline=spline,
                backward=backward,
                use_implement=waypoints[i].use_implement,
                stop_at_end=waypoints[i].stop_at_waypoint or is_last_segment,
            ))
        if self.reverse:
            forward_segments = list(path_segments)
            path_segments = []
            for j in range(len(forward_segments) - 1, -1, -1):
                seg = forward_segments[j]
                yaw_offset = 0.0 if seg.backward else math.pi
                is_last_reversed = j == 0
                path_segments.append(DriveSegment.from_poses(
                    Pose(x=seg.end.x, y=seg.end.y, yaw=seg.end.yaw + yaw_offset),
                    Pose(x=seg.start.x, y=seg.start.y, yaw=seg.start.yaw + yaw_offset),
                    backward=seg.backward,
                    use_implement=seg.use_implement,
                    stop_at_end=seg.stop_at_end or is_last_reversed,
                ))
        path_segments = skip_completed_segments(self.pose_provider.pose, path_segments,
                                                max_distance=self.RESUME_MAX_OFFSET, max_angle=self.RESUME_MAX_HEADING)
        if not path_segments:
            rosys.notify(
                f'Align the robot with the track (within {self.RESUME_MAX_OFFSET:.1f} m and '
                f'{np.rad2deg(self.RESUME_MAX_HEADING):.0f}°)',
                'negative', log_level=logging.ERROR)
        return path_segments

    async def approach_start(self) -> None:
        """Approaches the start of the track directly.

        Use with caution, alignment with the track will not be checked.
        If reverse is enabled, the robot will approach the end of the track instead.
        """
        recorded_track = self.recorded_track_provider.selected_track
        if recorded_track is None:
            raise ValueError('No track selected')
        start_index = -1 if self.reverse else 0
        start_pose = recorded_track.waypoints[start_index].pose.to_local()
        if self.reverse:
            start_pose = start_pose.rotate(math.pi)
        spline = Spline.from_poses(self.pose_provider.pose, start_pose)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
            await self.driver.drive_spline(spline)

    def settings_ui(self) -> None:
        super().settings_ui()
        self._recording_banner()  # type: ignore[call-arg]
        self._settings_content_row = ui.column().classes('w-full gap-2')
        with self._settings_content_row:
            self._settings_content()  # type: ignore[call-arg]
        # Stable host for dialogs so settings refreshes don't tear them down.
        self._dialog_host = ui.element('div')
        self._set_settings_visible(not self.track_recording_controller.is_recording)

    @ui.refreshable
    def _settings_content(self) -> None:
        provider = self.recorded_track_provider
        selected = provider.selected_track
        with ui.row():
            ui.select(
                value=selected.id if selected else None,
                options={t.id: t.name for t in provider.recorded_tracks},
                label='Select Recorded Track',
                on_change=lambda e: provider.select_track(e.value)) \
                .style('min-width: 300px')
            ui.button(icon='add', on_click=self.start_new_track_recording) \
                .tooltip('Create a new track and start recording')
        with ui.row():
            ui.button(icon='edit', on_click=self.start_track_editing) \
                .tooltip('Open dialog to edit selected track') \
                .bind_enabled_from(provider, 'selected_track', lambda t: t is not None)
            ui.button(icon='fiber_manual_record', on_click=self.resume_track_recording) \
                .tooltip('Resume recording into selected track') \
                .bind_enabled_from(provider, 'selected_track', lambda t: t is not None)
            ui.button(icon='moving', on_click=self._start_approach) \
                .tooltip('Approach start of selected track. Use with caution!') \
                .bind_enabled_from(provider, 'selected_track', lambda t: t is not None)
            ui.checkbox('Reverse direction').bind_value(self, 'reverse')

    @ui.refreshable
    def _recording_banner(self) -> None:
        active_track = self.track_recording_controller.active_track
        if active_track is None:
            self._banner_count_label = None
            return
        with ui.row().classes('w-full items-center bg-orange-100 rounded p-2 gap-2'):
            with ui.row():
                ui.icon('fiber_manual_record').classes('text-red-600 animate-pulse')
                self._banner_count_label = ui.label(self._banner_text(active_track)).classes('text-sm')
            with ui.row():
                ui.button('Open recorder', on_click=self.open_recorder).props('flat dense')
                ui.button('Stop recording', on_click=self.track_recording_controller.stop_recording, color='red') \
                    .props('flat dense')

    @staticmethod
    def _banner_text(active_track: RecordedTrack) -> str:
        return f'Recording: {active_track.name} ({len(active_track.waypoints)} waypoints)'

    def _update_banner_count(self) -> None:
        active_track = self.track_recording_controller.active_track
        label = self._banner_count_label
        if active_track is None or label is None or label.is_deleted:
            return
        label.set_text(self._banner_text(active_track))

    def _set_settings_visible(self, visible: bool) -> None:
        if self._settings_content_row is not None:
            self._settings_content_row.set_visibility(visible)

    def _start_approach(self) -> None:
        if self.automator is None:
            return
        self.automator.start(self.approach_start())

    async def start_new_track_recording(self) -> None:
        await self.track_recording_controller.start_recording(RecordedTrack())

    async def resume_track_recording(self) -> None:
        selected = self.recorded_track_provider.selected_track
        if selected is None:
            return
        await self.track_recording_controller.start_recording(selected)

    def start_track_editing(self) -> None:
        selected = self.recorded_track_provider.selected_track
        if selected is None:
            return
        self._open_recorder(selected)

    def open_recorder(self) -> None:
        active_track = self.track_recording_controller.active_track
        if active_track is None:
            return
        self._open_recorder(active_track)

    def _open_recorder(self, active_track: RecordedTrack) -> None:
        # Imported lazily to avoid an import cycle: the dialog lives in
        # ``interface.components`` which imports back into this package.
        # pylint: disable=import-outside-toplevel
        from ..interface.components.track_recorder_dialog import TrackRecorderDialog  # noqa: PLC0415
        # Build inside the stable dialog host so settings refreshes don't tear the dialog down.
        host = self._dialog_host if self._dialog_host is not None else contextlib.nullcontext()
        with host:
            self._current_recorder = TrackRecorderDialog(
                existing_track=active_track,
                recorded_track_provider=self.recorded_track_provider,
                track_recording_controller=self.track_recording_controller,
                pose_provider=self.pose_provider,
                gnss=self.gnss)
