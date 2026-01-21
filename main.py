#! /usr/bin/env python
import rosys
from nicegui import app, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Steerer, keyboard_control, robot_object

import feldfreund_devkit
from feldfreund_devkit.config import FeldfreundConfiguration, config_from_id
from feldfreund_devkit.implement import ImplementDummy
from feldfreund_devkit.navigation import StraightLineNavigation


class System(feldfreund_devkit.System):
    def __init__(self, config: FeldfreundConfiguration) -> None:
        super().__init__(config)
        self.steerer = Steerer(self.feldfreund.wheels, speed_scaling=0.25)
        self.driver = Driver(self.feldfreund.wheels, self.odometer, parameters=self.config.driver)
        self.shape = rosys.geometry.Prism.default_robot_shape()
        self.navigation = StraightLineNavigation(implement=ImplementDummy(),
                                                 driver=self.driver,
                                                 pose_provider=self.odometer)
        self.automator = Automator(self.steerer, on_interrupt=self.feldfreund.stop, notify=False)
        self.automator.default_automation = self.navigation.start

        @ui.page('/')
        def ui_content() -> None:
            keyboard_control(self.steerer)
            with ui.scene():
                robot_object(self.shape, self.odometer)
            with ui.card():
                ui.label('hold SHIFT to steer with the keyboard arrow keys or use the automation controls')
                with ui.row():
                    self.navigation.settings_ui()
                with ui.row():
                    automation_controls(self.automator)


def startup() -> None:
    config = config_from_id('example')
    System(config).persistent()


app.on_startup(startup)

ui.run(title='Feldfreund_devkit')
