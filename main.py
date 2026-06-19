#! /usr/bin/env python
import rosys
from nicegui import app, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Steerer, keyboard_control, robot_object

import feldfreund_devkit
from feldfreund_devkit.config import FeldfreundConfiguration, Secrets, config_from_id
from feldfreund_devkit.implement import ImplementDummy
from feldfreund_devkit.navigation import (
    RecordedTrackNavigation,
    RecordedTrackProvider,
    StraightLineNavigation,
    TrackRecordingController,
    WaypointNavigation,
)


class System(feldfreund_devkit.System):
    def __init__(self, config: FeldfreundConfiguration, secrets: Secrets) -> None:
        super().__init__(config, secrets=secrets)
        self.steerer = Steerer(self.feldfreund.wheels, speed_scaling=0.25)
        self.driver = Driver(self.feldfreund.wheels, self.odometer, parameters=self.config.driver)
        self.shape = rosys.geometry.Prism.default_robot_shape()
        self.automator = Automator(self.steerer, on_interrupt=self.feldfreund.stop, notify=False)

        self.recorded_track_provider = RecordedTrackProvider().persistent()
        self.track_recording_controller = TrackRecordingController(
            self.recorded_track_provider, pose_provider=self.odometer, gnss=self.feldfreund.gnss)

        common = {'implement': ImplementDummy(), 'driver': self.driver, 'pose_provider': self.odometer}
        self.navigations: dict[str, WaypointNavigation] = {n.name: n for n in [
            StraightLineNavigation(**common),
            RecordedTrackNavigation(recorded_track_provider=self.recorded_track_provider,
                                    track_recording_controller=self.track_recording_controller,
                                    gnss=self.feldfreund.gnss,
                                    automator=self.automator,
                                    **common),
        ]}
        self._current_navigation: WaypointNavigation = next(iter(self.navigations.values()))
        self.automator.default_automation = self._current_navigation.start

    @property
    def current_navigation(self) -> WaypointNavigation:
        return self._current_navigation

    @current_navigation.setter
    def current_navigation(self, navigation: WaypointNavigation) -> None:
        self._current_navigation = navigation
        self.automator.default_automation = navigation.start


def startup() -> None:
    secrets = Secrets()
    config = config_from_id('example', secrets=secrets)
    system = System(config, secrets).persistent()

    @ui.page('/')
    def ui_content() -> None:
        keyboard_control(system.steerer)
        with ui.scene():
            robot_object(system.shape, system.odometer)

        @ui.refreshable
        def navigation_settings() -> None:
            system.current_navigation.settings_ui()

        def select_navigation(name: str) -> None:
            system.current_navigation = system.navigations[name]
            navigation_settings.refresh()

        with ui.card():
            ui.label('hold SHIFT to steer with the keyboard arrow keys or use the automation controls')
            ui.select(list(system.navigations), value=system.current_navigation.name, label='Navigation',
                      on_change=lambda e: select_navigation(e.value)).classes('w-64')
            with ui.row():
                navigation_settings()
            with ui.row():
                automation_controls(system.automator)


app.on_startup(startup)

ui.run(title='Feldfreund_devkit')
