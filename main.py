#! /usr/bin/env python
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, NoReturn

import rosys
from nicegui import app, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Steerer, keyboard_control, robot_object
from rosys.geometry import Point, Point3d, Pose3d

import feldfreund_devkit
from feldfreund_devkit import Implement, RobotLocator, WorkContext
from feldfreund_devkit.config import FeldfreundConfiguration, ImplementConfiguration, Secrets, config_from_id
from feldfreund_devkit.navigation import (
    CannotStop,
    PathDriver,
    RecordedTrackNavigation,
    RecordedTrackProvider,
    StraightLineNavigation,
    TrackRecordingController,
    drive_and_work,
)

DemoNavigation = StraightLineNavigation | RecordedTrackNavigation


@dataclass
class IntervalRun:
    """What the demo tool keeps for one run."""

    stops: int = 0


class IntervalImplement(Implement[IntervalRun]):
    """A demo tool that comes to rest every ``interval`` metres and holds there for ``dwell`` seconds."""

    def __init__(self, *, interval: float = 0.5, dwell: float = 1.0) -> None:
        super().__init__(ImplementConfiguration(lizard_name='None', display_name='Interval',
                                                offset=Pose3d(x=0.3), work_radius=0.0))
        self.interval = interval
        self.dwell = dwell

    @property
    def modules(self) -> list[rosys.hardware.Module]:
        return []

    async def stop(self) -> None:
        pass

    @asynccontextmanager
    async def activated(self) -> AsyncGenerator[IntervalRun, None]:  # pylint: disable=invalid-overridden-method
        run = IntervalRun()
        try:
            yield run
        finally:
            rosys.notify(f'{self.name} stopped {run.stops} times')

    async def work(self, ctx: WorkContext, context: IntervalRun) -> NoReturn:
        while True:
            target = ctx.locator.pose.transform(Point(x=self.offset.x + self.interval, y=0.0))
            try:
                async with ctx.motion.stop_over(target, self.offset.x):
                    context.stops += 1
                    await rosys.sleep(self.dwell)
            except CannotStop:  # We have already driven over the stop, skip it
                await rosys.sleep(0.1)

    def can_reach(self, local_point: Point3d) -> bool:
        return True

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...


class System(feldfreund_devkit.System):
    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, config: FeldfreundConfiguration, secrets: Secrets) -> None:
        super().__init__(config, secrets=secrets)
        self.robot_locator = RobotLocator(self.feldfreund.wheels,
                                          gnss=self.feldfreund.gnss,
                                          imu=self.feldfreund.imu,
                                          gnss_config=self.config.gnss)
        self.steerer = Steerer(self.feldfreund.wheels, speed_scaling=0.25)
        self.driver = Driver(self.feldfreund.wheels, self.robot_locator, parameters=self.config.driver)
        self.shape = rosys.geometry.Prism.default_robot_shape()
        self.automator = Automator(self.steerer, on_interrupt=self.feldfreund.stop, notify=False)

        self.recorded_track_provider = RecordedTrackProvider().persistent()
        self.track_recording_controller = TrackRecordingController(
            self.recorded_track_provider, pose_provider=self.robot_locator, gnss=self.feldfreund.gnss)

        self.path_driver = PathDriver(self.driver)
        self.implement = IntervalImplement()
        self.navigations: dict[str, DemoNavigation] = {
            'Straight Line': StraightLineNavigation(self.robot_locator),
            'Recorded Track': RecordedTrackNavigation(
                recorded_track_provider=self.recorded_track_provider,
                track_recording_controller=self.track_recording_controller,
                gnss=self.feldfreund.gnss,
                pose_provider=self.robot_locator),
        }
        self.navigation_name = next(iter(self.navigations))
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT
        self.automator.default_automation = self._drive

    @property
    def navigation(self) -> DemoNavigation:
        return self.navigations[self.navigation_name]

    async def _drive(self) -> None:
        async with self.implement.activated() as context:
            await drive_and_work(self.navigation, self.path_driver, self.robot_locator,
                                 speed_limit=self.linear_speed_limit,
                                 implement=self.implement, context=context)

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {'linear_speed_limit': self.linear_speed_limit}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)


def speed_setting(system: System) -> None:
    minimum = system.driver.parameters.throttle_at_end_min_speed
    maximum = system.driver.parameters.linear_speed_limit
    ui.number('Linear Speed', step=0.01, min=minimum, max=maximum, format='%.2f', suffix='m/s',
              on_change=system.request_backup) \
        .bind_value(system, 'linear_speed_limit') \
        .props('dense outlined') \
        .classes('w-24') \
        .tooltip(f'Forward speed limit between {minimum:.2f} and {maximum:.2f} m/s '
                 f'(default: {System.LINEAR_SPEED_LIMIT:.2f})')


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
            with ui.row().classes('items-center'):
                speed_setting(system)
                automation_controls(system.automator)


app.on_startup(startup)

ui.run(title='Feldfreund_devkit')
