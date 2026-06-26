from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

import rosys
from nicegui import ui
from nicegui.elements.leaflet.leaflet_layers import GenericLayer, Marker
from rosys.driving.driver import PoseProvider
from rosys.geometry import GeoPose
from rosys.hardware import Gnss

from ...navigation.recorded_track import (
    GnssRequirement,
    RecordedTrack,
    RecordedTrackProvider,
    RecordedWaypoint,
)
from ...navigation.track_recording_controller import TrackRecordingController
from .confirm_dialog import ConfirmDialog as confirm_dialog

log = logging.getLogger('feldfreund_devkit.track_recorder')

TRACK_COLOR_DEFAULT = '#6E93D6'
TRACK_COLOR_USE_IMPLEMENT = '#F2C037'
TRACK_COLOR_STOPP_AT_WP = '#D6716E'


def _marker_style(wp: RecordedWaypoint, *, highlighted: bool = False) -> dict[str, Any]:
    color = (
        TRACK_COLOR_STOPP_AT_WP if wp.stop_at_waypoint else
        TRACK_COLOR_USE_IMPLEMENT if wp.use_implement else
        TRACK_COLOR_DEFAULT
    )
    return {
        'color': color,
        'fillColor': color,
        'fillOpacity': 0.95 if highlighted else 0.8,
        'radius': 10 if highlighted else 6,
    }


class RecordedTrackList:
    """Side panel showing the waypoints in the active recording, with row controls."""

    def __init__(
        self,
        recorded_track: RecordedTrack,
        provider: RecordedTrackProvider,
        on_change: Callable[[], None] | None = None,
        on_select: Callable[[int], None] | None = None,
    ) -> None:
        self.recorded_track = recorded_track
        self.provider = provider
        self._on_change = on_change
        self._on_select = on_select
        self._rows: list[ui.row] = []
        self._selected_index: int | None = None
        self._scroll_percent: float = 1.0

        self.container = ui.column().classes('w-full h-full gap-1')
        with self.container:
            self._scroll_area = ui.scroll_area(on_scroll=self._on_scroll).classes('flex-grow w-full')
            with ui.row().classes('w-full justify-center'):
                self._waypoint_count_label = ui.label().classes('text-sm text-gray-600')
        self._render_waypoints()

    def _on_scroll(self, e) -> None:
        self._scroll_percent = e.vertical_percentage

    def select(self, index: int) -> None:
        """Select a waypoint from outside (e.g. map click). Scrolls into view."""
        if not 0 <= index < len(self._rows):
            return
        self._selected_index = index
        for r in self._rows:
            r.classes(remove='bg-blue-100')
        self._rows[index].classes(add='bg-blue-100')
        if len(self._rows) > 1:
            self._scroll_area.scroll_to(percent=index / max(1, len(self._rows) - 1))

    def notify_waypoints_changed(self) -> None:
        was_at_bottom = self._scroll_percent > 0.9
        self._render_waypoints()
        self._apply_selection()
        if was_at_bottom or self._selected_index is None:
            self._scroll_area.scroll_to(percent=1.0)
        else:
            self._scroll_area.scroll_to(percent=self._scroll_percent)

    def _render_waypoints(self) -> None:
        self._rows.clear()
        self._scroll_area.clear()
        with self._scroll_area:
            if self.recorded_track.waypoints:
                for i, wp in enumerate(self.recorded_track.waypoints):
                    self._create_waypoint_row(i, wp)
            else:
                with ui.column().classes('w-full items-center text-center gap-2 py-8 text-gray-500'):
                    ui.icon('place', size='lg')
                    ui.label('Drive the robot, then press Add Waypoint (or Space) to record your first point.') \
                        .classes('text-sm')
        self._waypoint_count_label.set_text(f'{len(self.recorded_track.waypoints)} waypoints')

    def _create_waypoint_row(self, index: int, wp: RecordedWaypoint) -> None:
        n = len(self.recorded_track.waypoints)
        row = ui.row().classes(
            'w-full items-center gap-1 cursor-pointer rounded px-2 py-1 hover:bg-gray-100'
        )
        self._rows.append(row)
        with row:
            row.on('click', lambda idx=index, r=row: self._select_waypoint(idx, r))
            ui.label(f'{index + 1}').classes('text-sm font-bold w-6 text-right')
            with ui.row().classes('flex-grow gap-1 items-center'):
                self._chip(wp.approach_reverse, 'arrow_back', 'Reverse',
                           lambda v: self._set_approach_reverse(index, v),
                           tooltip='Drive backwards to this waypoint')
                self._chip(wp.use_implement, 'construction', 'Implement',
                           lambda v: self._set_use_implement(index, v),
                           tooltip='Allow implement usage on the segment to this waypoint')
                self._chip(wp.stop_at_waypoint, 'stop_circle', 'Stop',
                           lambda v: self._set_stop_at_waypoint(index, v),
                           tooltip='Stop at this waypoint')
                ui.button(icon='arrow_upward', on_click=lambda idx=index: self._move_up(idx)) \
                    .props('flat round dense size=sm') \
                    .tooltip('Move up') \
                    .set_enabled(index > 0)
                ui.button(icon='arrow_downward', on_click=lambda idx=index: self._move_down(idx)) \
                    .props('flat round dense size=sm') \
                    .tooltip('Move down') \
                    .set_enabled(index < n - 1)
                ui.button(icon='delete', on_click=lambda idx=index: self._delete_waypoint(idx)) \
                    .props('flat round dense color=red size=sm') \
                    .tooltip('Delete waypoint')

    @staticmethod
    def _chip(value: bool, icon: str, label: str, on_toggle: Callable[[bool], None], *, tooltip: str) -> None:
        chip = ui.chip(label, icon=icon, color='primary' if value else 'grey-3',
                       text_color='white' if value else 'grey-8') \
            .props('clickable dense')
        chip.tooltip(tooltip)
        state = {'value': value}

        def _toggle(_=None) -> None:
            state['value'] = not state['value']
            on_toggle(state['value'])
            chip.props(f'color={"primary" if state["value"] else "grey-3"}')
            chip.props(f'text-color={"white" if state["value"] else "grey-8"}')

        chip.on('click', _toggle)

    def _move_up(self, index: int) -> None:
        if index <= 0:
            return
        self.recorded_track.move_waypoint(index, index - 1)
        self.provider.notify_track_modified()
        if self._selected_index == index:
            self._selected_index = index - 1
        elif self._selected_index == index - 1:
            self._selected_index = index
        self._notify_change()

    def _move_down(self, index: int) -> None:
        if index >= len(self.recorded_track.waypoints) - 1:
            return
        self.recorded_track.move_waypoint(index, index + 1)
        self.provider.notify_track_modified()
        if self._selected_index == index:
            self._selected_index = index + 1
        elif self._selected_index == index + 1:
            self._selected_index = index
        self._notify_change()

    def _select_waypoint(self, index: int, row: ui.row) -> None:
        self._selected_index = index
        for r in self._rows:
            r.classes(remove='bg-blue-100')
        row.classes(add='bg-blue-100')
        if self._on_select:
            self._on_select(index)

    def _apply_selection(self, *, notify_map: bool = False) -> None:
        if self._selected_index is not None and 0 <= self._selected_index < len(self._rows):
            self._rows[self._selected_index].classes(add='bg-blue-100')
            if notify_map and self._on_select:
                self._on_select(self._selected_index)

    def _set_approach_reverse(self, index: int, value: bool) -> None:
        self.recorded_track.set_waypoint_approach_reverse(index, value)
        self.provider.notify_track_modified()
        if self._on_change:
            self._on_change()

    def _set_use_implement(self, index: int, value: bool) -> None:
        self.recorded_track.set_waypoint_use_implement(index, value)
        self.provider.notify_track_modified()
        if self._on_change:
            self._on_change()

    def _set_stop_at_waypoint(self, index: int, value: bool) -> None:
        self.recorded_track.set_waypoint_stop_at_waypoint(index, value)
        self.provider.notify_track_modified()
        if self._on_change:
            self._on_change()

    def _delete_waypoint(self, index: int) -> None:
        self.recorded_track.remove_waypoint(index)
        self.provider.notify_track_modified()
        if self._selected_index is not None:
            if self._selected_index == index:
                self._selected_index = None
            elif self._selected_index > index:
                self._selected_index -= 1
        self._notify_change()

    def _notify_change(self) -> None:
        saved_percent = self._scroll_percent
        self._render_waypoints()
        if self._on_change:
            self._on_change()
        self._apply_selection(notify_map=True)
        self._scroll_area.scroll_to(percent=saved_percent)


class TrackRecorderDialog:

    def __init__(self, existing_track: RecordedTrack, *,
                 recorded_track_provider: RecordedTrackProvider,
                 track_recording_controller: TrackRecordingController,
                 pose_provider: PoseProvider,
                 gnss: Gnss | None = None,
                 robot_marker_icon_url: str | None = None) -> None:
        self.provider = recorded_track_provider
        self.controller = track_recording_controller
        self.pose_provider = pose_provider
        self.gnss = gnss
        # URL of the robot marker image (served by the app, e.g. 'assets/robot.png').
        # When None the map shows Leaflet's default marker.
        self.robot_marker_icon_url = robot_marker_icon_url

        self.recorded_track = existing_track

        self.dialog: ui.dialog = None  # type: ignore[assignment]
        self.dialog_map: ui.leaflet = None  # type: ignore[assignment]
        self.robot_marker: Marker | None = None
        self.recorded_track_ui: RecordedTrackList = None  # type: ignore[assignment]
        self._elapsed_label: ui.label | None = None
        self._count_label: ui.label = None  # type: ignore[assignment]
        self._gnss_pill: ui.chip | None = None
        self._add_button: ui.button | None = None
        self._undo_button: ui.button | None = None
        self._recording_action_row: ui.row | None = None
        self._view_action_row: ui.row | None = None

        self._track_segments: list[GenericLayer] = []
        self._waypoint_markers: list[GenericLayer] = []
        self._tick_timer: ui.timer | None = None

        self._build_dialog()
        self._update_track_on_map()
        self._update_status()
        self._apply_mode()
        self.dialog.open()

        self.controller.WAYPOINT_ADDED.subscribe(self._on_waypoint_added)
        self.controller.RECORDING_STARTED.subscribe(self._on_recording_started)
        self.controller.RECORDING_STOPPED.subscribe(self._on_recording_stopped)
        self.dialog.on('hide', self.tear_down)

    @property
    def is_recording(self) -> bool:
        return self.controller.is_recording and self.controller.active_track is self.recorded_track

    @property
    def current_position(self) -> GeoPose:
        return GeoPose.from_pose(self.pose_provider.pose)

    def _unsubscribe(self) -> None:
        self.controller.WAYPOINT_ADDED.unsubscribe(self._on_waypoint_added)
        self.controller.RECORDING_STARTED.unsubscribe(self._on_recording_started)
        self.controller.RECORDING_STOPPED.unsubscribe(self._on_recording_stopped)

    def tear_down(self) -> None:
        """Drop controller subscriptions and stop the tick timer.

        The DOM (dialog, map, keyboard) is removed by the owner clearing the dialog
        host; ``ui.timer`` even cancels itself on deletion, but ``hide`` does not
        delete anything, so the timer is cancelled explicitly here. Idempotent —
        both ``unsubscribe`` and ``Timer.cancel`` are no-ops once already done.
        """
        self._unsubscribe()
        if self._tick_timer is not None:
            self._tick_timer.cancel()

    def _build_dialog(self) -> None:
        with ui.dialog() as self.dialog, \
                ui.card().style('width: 1100px; max-width: 95vw; padding: 12px'):
            self._build_status_bar()
            with ui.row().classes('w-full no-wrap gap-3').style('height: 460px'):
                with ui.column().classes('h-full').style('flex: 0 0 60%'):
                    self._build_map()
                with ui.column().classes('h-full').style('flex: 0 0 38%'):
                    self.recorded_track_ui = RecordedTrackList(
                        self.recorded_track, self.provider,
                        on_change=self._update_track_on_map,
                        on_select=self._highlight_waypoint,
                    )
            ui.separator()
            self._build_action_bar()
        ui.keyboard(on_key=self._on_key)

    def _build_status_bar(self) -> None:
        with ui.row().classes('w-full items-center gap-3 px-1'):
            ui.input(value=self.recorded_track.name) \
                .on_value_change(lambda e: setattr(self.recorded_track, 'name', e.value)) \
                .props('dense outlined') \
                .classes('w-64') \
                .tooltip('Track name')
            self._elapsed_label = ui.label('0:00').classes('text-sm font-mono text-gray-600')
            self._count_label = ui.label('0 waypoints').classes('text-sm text-gray-600')
            self._gnss_pill = ui.chip('', icon='gps_fixed').props('dense')
            ui.space()
            ui.button(icon='close', on_click=self.dialog.close) \
                .props('flat round dense') \
                .tooltip('Hide dialog — recording continues. Use the recorded-track settings panel to stop or reopen.')

    def _build_map(self) -> None:
        self.dialog_map = ui.leaflet(
            center=self.current_position.point.degree_tuple,
            zoom=18,
        ).classes('w-full h-full')
        self.dialog_map.tile_layer(
            url_template=r'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            options={
                'maxZoom': 24,
                'maxNativeZoom': 18,
                'attribution': '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
            },
        )
        self.update_robot_position()
        self._tick_timer = ui.timer(1, self._tick)

    def _build_action_bar(self) -> None:
        with ui.row().classes('w-full items-center gap-3 px-1'):
            ui.select(
                {GnssRequirement.NONE: 'None', GnssRequirement.GNSS: 'GNSS', GnssRequirement.RTK: 'RTK'},
                value=self.recorded_track.gnss_requirement,
                label='Min. GPS',
                on_change=self._on_gnss_requirement_changed,
            ).classes('w-32')
            ui.button(icon='delete', color='red', on_click=self._delete_track) \
                .tooltip('Delete this track.')
            self._recording_action_row = ui.row().classes('items-center gap-3 flex-grow')
            with self._recording_action_row:
                ui.space()
                self._undo_button = ui.button('Undo last', icon='undo', on_click=self._undo_last) \
                    .props('flat') \
                    .tooltip('Remove the most recently added waypoint')
                self._add_button = ui.button('Add Waypoint', icon='add_location_alt', on_click=self._add_waypoint)
                if self.gnss is not None:
                    self._add_button.bind_enabled_from(
                        self.gnss, 'last_measurement',
                        backward=lambda m: self.recorded_track.meets_gnss_requirement(m.gps_quality if m else None),
                    )
                ui.button('Finish recording', icon='stop', on_click=self._finish) \
                    .props('outline color=red')
            self._view_action_row = ui.row().classes('items-center gap-3 flex-grow')
            with self._view_action_row:
                ui.space()
                ui.button('Start Recording', icon='fiber_manual_record', on_click=self._start_recording)

    def _tick(self) -> None:
        self.update_robot_position()
        self._update_status()

    def _update_status(self) -> None:
        if self._elapsed_label is None:
            return
        seconds = int(self.controller.elapsed_seconds)
        self._elapsed_label.set_text(f'{seconds // 60}:{seconds % 60:02d}')
        n = len(self.recorded_track.waypoints)
        self._count_label.set_text(f'{n} waypoint{"s" if n != 1 else ""}')
        if self._undo_button is not None:
            self._undo_button.set_enabled(n > 0)
        self._update_gnss_pill()

    def _update_gnss_pill(self) -> None:
        if self._gnss_pill is None:
            return
        if self.gnss is None:
            self._gnss_pill.set_text('GNSS not configured')
            self._gnss_pill.props('color=grey-5 text-color=white')
            return
        meets = self._meets_gnss_requirement
        quality = self.gnss.last_measurement.gps_quality if self.gnss.last_measurement else None
        label = quality.name if quality is not None else 'NO FIX'
        self._gnss_pill.set_text(label)
        self._gnss_pill.props(f'color={"positive" if meets else "negative"} text-color=white')

    def update_robot_position(self) -> None:
        geo_pose = GeoPose.from_pose(self.pose_provider.pose)
        latlng = geo_pose.point.degree_tuple
        if self.robot_marker is None:
            self.robot_marker = self.dialog_map.marker(latlng=latlng)
            if self.robot_marker_icon_url is not None:
                icon = f'L.icon({{iconUrl: "{self.robot_marker_icon_url}", iconSize: [50,50], iconAnchor: [20,20]}})'
                self.robot_marker.run_method(':setIcon', icon)
        self.robot_marker.move(*latlng)

    def _on_key(self, e) -> None:
        if not (self.dialog.value and e.action.keydown and not e.action.repeat):
            return
        if e.key.name != ' ':
            return
        if not self._meets_gnss_requirement:
            return
        rosys.background_tasks.create(self._add_waypoint(), name='add waypoint via space')

    @property
    def _meets_gnss_requirement(self) -> bool:
        if self.gnss is None:
            return True
        measurement = self.gnss.last_measurement
        gps_quality = measurement.gps_quality if measurement else None
        return self.recorded_track.meets_gnss_requirement(gps_quality)

    async def _add_waypoint(self) -> None:
        if not self._meets_gnss_requirement:
            rosys.notify('GNSS quality insufficient', 'negative')
            return
        await self.controller.add_waypoint_at_current_pose()

    def _undo_last(self) -> None:
        if not self.recorded_track.waypoints:
            return
        self.recorded_track.remove_waypoint(len(self.recorded_track.waypoints) - 1)
        self.provider.notify_track_modified()
        self.recorded_track_ui.notify_waypoints_changed()
        self._update_track_on_map()
        self._update_status()

    def _on_gnss_requirement_changed(self, e) -> None:
        self.recorded_track.gnss_requirement = e.value
        self.provider.notify_track_modified()

    def _on_waypoint_added(self) -> None:
        self._update_track_on_map()
        self.recorded_track_ui.notify_waypoints_changed()
        self._update_status()

    def _on_recording_started(self) -> None:
        self._apply_mode()
        self._update_status()

    def _on_recording_stopped(self) -> None:
        self._apply_mode()
        self._update_status()

    def _apply_mode(self) -> None:
        if self._recording_action_row is not None:
            self._recording_action_row.set_visibility(self.is_recording)
        if self._view_action_row is not None:
            self._view_action_row.set_visibility(not self.is_recording)

    def _start_recording(self) -> None:
        if self.is_recording:
            return
        rosys.background_tasks.create(
            self.controller.start_recording(self.recorded_track),
            name='start track recording from dialog',
        )

    async def _delete_track(self) -> None:
        active = self.controller.active_track
        if active is not None and active.id == self.recorded_track.id:
            rosys.notify('Cannot delete a track while it is being recorded', 'warning')
            return
        if not await confirm_dialog('Are you sure you want to delete this track?'):
            return
        self.provider.remove_recorded_track(self.recorded_track.id)
        self.dialog.close()

    async def _finish(self) -> None:
        await self.controller.stop_recording()

    def _highlight_waypoint(self, index: int) -> None:
        for i, marker in enumerate(self._waypoint_markers):
            wp = self.recorded_track.waypoints[i]
            marker.run_method('setStyle', _marker_style(wp, highlighted=i == index))

    def _update_track_on_map(self) -> None:
        for s in self._track_segments:
            self.dialog_map.remove_layer(s)
        self._track_segments.clear()
        for m in self._waypoint_markers:
            self.dialog_map.remove_layer(m)
        self._waypoint_markers.clear()

        waypoints = self.recorded_track.waypoints
        for i in range(1, len(waypoints)):
            prev_wp = waypoints[i - 1]
            curr_wp = waypoints[i]
            color = (TRACK_COLOR_USE_IMPLEMENT if curr_wp.use_implement
                     else TRACK_COLOR_DEFAULT)
            options: dict[str, Any] = {'color': color, 'weight': 3}
            if curr_wp.approach_reverse:
                options['dashArray'] = '6,6'
            segment = self.dialog_map.generic_layer(
                name='polyline',
                args=[[prev_wp.degree_tuple, curr_wp.degree_tuple], options],
            )
            self._track_segments.append(segment)

        for _i, wp in enumerate(waypoints):
            marker = self.dialog_map.generic_layer(
                name='circleMarker',
                args=[list(wp.degree_tuple), _marker_style(wp)],
            )
            self._waypoint_markers.append(marker)
