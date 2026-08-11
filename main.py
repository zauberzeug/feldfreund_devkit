#! /usr/bin/env python
import rosys
from nicegui import app, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Steerer, keyboard_control, robot_object

import feldfreund_devkit
from feldfreund_devkit import NoDetection, no_work
from feldfreund_devkit.config import FeldfreundConfiguration, Secrets, config_from_id
from feldfreund_devkit.navigation import (
    PathDriver,
    RecordedTrackProvider,
    StraightLineNavigation,
    TrackRecordingController,
    drive_and_work,
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

        self.path_driver = PathDriver(self.driver)
        self.route = StraightLineNavigation(self.odometer)
        self.automator.default_automation = self._drive

    async def _drive(self) -> None:
        self.path_driver.ambient_limit = lambda: self.route.linear_speed_limit
        await drive_and_work(self.route, self.path_driver, self.odometer,
                             detection=NoDetection(), work=no_work)


def startup() -> None:
    secrets = Secrets()
    config = config_from_id('example', secrets=secrets)
    system = System(config, secrets).persistent()

    @ui.page('/')
    def ui_content() -> None:
        keyboard_control(system.steerer)
        with ui.scene():
            robot_object(system.shape, system.odometer)

        with ui.card():
            ui.label('hold SHIFT to steer with the keyboard arrow keys or use the automation controls')
            with ui.row():
                system.route.settings_ui()
            with ui.row():
                automation_controls(system.automator)


app.on_startup(startup)

ui.run(title='Feldfreund_devkit')
