import logging
from contextlib import aclosing

import rosys
from nicegui import Event
from rosys.analysis import track
from rosys.driving import Driver

from .drive_segment import DriveSegment
from .navigation import Navigation
from .path_driver import PathDriver


class Orchestrator:
    """Runs one automation: pulls the navigation's route and drives it.

    Owns the run rather than the route, so the navigation stays a planner and the ``PathDriver``
    stays a driver. What is being driven right now is visible here, since a pulled route has no
    other place that knows.
    """

    def __init__(self, navigation: Navigation, driver: Driver) -> None:
        self.log = logging.getLogger('feldfreund.orchestrator')
        self.navigation = navigation
        self.path_driver = PathDriver(driver, speed_limit=lambda: navigation.linear_speed_limit)
        self.current_segment: DriveSegment | None = None

        self.SEGMENT_STARTED = Event[DriveSegment]()
        """driving a segment has begun (argument: ``DriveSegment``)"""

        self.SEGMENT_COMPLETED = Event[DriveSegment]()
        """a segment has been driven to its end (argument: ``DriveSegment``)"""

        self.RUN_COMPLETED = Event[[]]()
        """the route ended and the robot came to rest"""

    @track
    async def run(self) -> None:
        """Drive the whole route.

        Stopping the wheels happens here rather than in a branch of the drive, because that is the
        one place cleanup may still await.
        """
        try:
            async with aclosing(self.navigation.segments()) as segments:
                async for segment in segments:
                    self.current_segment = segment
                    self.SEGMENT_STARTED.emit(segment)
                    await self.path_driver.drive(segment)
                    self.SEGMENT_COMPLETED.emit(segment)
            self.RUN_COMPLETED.emit()
            rosys.notify('Automation finished', 'positive')
        finally:
            self.current_segment = None
            await self.path_driver.driver.wheels.stop()
