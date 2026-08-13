#! /usr/bin/env python
import rosys
from nicegui import app, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Steerer, keyboard_control, robot_object

import feldfreund_devkit
from feldfreund_devkit import ImplementDummy
from feldfreund_devkit.config import FeldfreundConfiguration, Secrets, config_from_id
from feldfreund_devkit.navigation import (
    PathDriver,
    RecordedTrackNavigation,
    RecordedTrackProvider,
    StraightLineNavigation,
    TrackRecordingController,
    drive_and_work,
)

DemoNavigation = StraightLineNavigation | RecordedTrackNavigation


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

        self.path_driver = PathDriver(self.driver)
        self.implement = ImplementDummy()
        self.navigations: dict[str, DemoNavigation] = {
            'Straight Line': StraightLineNavigation(self.odometer),
            'Recorded Track': RecordedTrackNavigation(
                recorded_track_provider=self.recorded_track_provider,
                track_recording_controller=self.track_recording_controller,
                gnss=self.feldfreund.gnss,
                automator=self.automator,
                driver=self.driver,
                pose_provider=self.odometer),
        }
        self.navigation_name = next(iter(self.navigations))
        self.automator.default_automation = self._drive

    @property
    def navigation(self) -> DemoNavigation:
        return self.navigations[self.navigation_name]

    async def _drive(self) -> None:
        await drive_and_work(self.navigation, self.path_driver, self.odometer,
                             speed_limit=self.driver.parameters.linear_speed_limit,
                             implement=self.implement, context=None)


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
            system.navigation.settings_ui()

        with ui.card():
            ui.label('hold SHIFT to steer with the keyboard arrow keys or use the automation controls')
            ui.select(list(system.navigations), label='Navigation',
                      on_change=lambda _: navigation_settings.refresh()) \
                .bind_value(system, 'navigation_name') \
                .classes('w-64')
            with ui.row():
                navigation_settings()
            with ui.row():
                automation_controls(system.automator)


app.on_startup(startup)

ui.run(title='Feldfreund_devkit')
