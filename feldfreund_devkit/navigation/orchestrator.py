import logging
from collections.abc import AsyncIterator
from contextlib import aclosing

import rosys
from nicegui import Event
from rosys.analysis import track
from rosys.driving import Driver
from rosys.driving.pose_provider import PoseProvider

from ..implement import ImplementException
from .drive_segment import DriveSegment
from .navigation import Navigation
from .path_driver import PathDriver
from .work_context import Detection, WorkContext, WorkFunction


class Orchestrator:
    """Runs one automation: pulls the navigation's route, drives it, and lets a tool work along it.

    Owns the run rather than the route, so the navigation stays a planner and the ``PathDriver``
    stays a driver.

    The tool's work runs alongside the drive for as long as the route stays workable -- a *working
    stretch* -- and is cancelled when the first non-working segment comes up. A tool therefore never
    runs during a headland turn, and a tool that works while moving is never interrupted at a
    segment boundary in the middle of a row.

    :param navigation: produces the route
    :param driver: the low-level driver executing velocities
    :param pose_provider: where the robot is, handed to the tool
    :param detection: controls when the robot looks for what it works on
    :param work: the tool's work loop; pass :func:`no_work` for a run that only drives
    """

    def __init__(self, navigation: Navigation, driver: Driver, pose_provider: PoseProvider, *,
                 detection: Detection, work: WorkFunction) -> None:
        self.log = logging.getLogger('feldfreund.orchestrator')
        self.navigation = navigation
        self.path_driver = PathDriver(driver, speed_limit=lambda: navigation.linear_speed_limit)
        self._pose_provider = pose_provider
        self._detection = detection
        self._work = work

        self.SEGMENT_STARTED = Event[DriveSegment]()
        """driving a segment has begun (argument: ``DriveSegment``)"""

        self.SEGMENT_COMPLETED = Event[DriveSegment]()
        """a segment has been driven to its end (argument: ``DriveSegment``)"""

        self.RUN_COMPLETED = Event[[]]()
        """the route ended and the robot came to rest"""

    @track
    async def run(self) -> None:
        """Drive the whole route, working the stretches that are workable.

        Stopping the wheels happens here rather than in a branch of the drive, because that is the
        one place cleanup may still await: on the error path ``parallelize`` closes its branches
        with ``GeneratorExit``, under which awaiting is illegal.
        """
        try:
            async with aclosing(self.navigation.segments()) as segments:
                route = _Route(segments)
                while (segment := await route.current()) is not None:
                    if segment.use_implement:
                        await rosys.automation.parallelize(
                            self._drive_stretch(route),
                            self._work_until_cancelled(),
                            return_when_first_completed=True,
                        )
                        continue
                    await self._drive(segment)
                    route.advance()
            self.RUN_COMPLETED.emit()
            rosys.notify('Automation finished', 'positive')
        finally:
            await self.path_driver.driver.wheels.stop()

    async def _drive_stretch(self, route: '_Route') -> None:
        """Drive workable segments until the one after them is not."""
        while (segment := await route.current()) is not None and segment.use_implement:
            await self._drive(segment)
            route.advance()

    async def _work_until_cancelled(self) -> None:
        """Run the tool's work loop for one stretch, and refuse to let it end the stretch itself.

        Returning early would look to ``parallelize`` like the stretch being over and halt the drive
        mid-row, so it is reported rather than obeyed.
        """
        await self._work(WorkContext(motion=self.path_driver, pose=self._pose_provider,
                                     detection=self._detection))
        raise ImplementException('the work loop returned; it must run until the stretch ends')

    async def _drive(self, segment: DriveSegment) -> None:
        self.SEGMENT_STARTED.emit(segment)
        await self.path_driver.drive(segment)
        self.SEGMENT_COMPLETED.emit(segment)


class _Route:
    """The segments to drive, one at a time.

    A stretch of workable segments ends at the first one that is not, which can only be found by
    pulling it -- so the run loop must be able to look at a segment before committing to drive it.
    Pulling still happens only once the previous segment has been driven, so a navigation that plans
    lazily still plans each segment at the moment it starts.
    """

    def __init__(self, segments: AsyncIterator[DriveSegment]) -> None:
        self._segments = segments
        self._current: DriveSegment | None = None

    async def current(self) -> DriveSegment | None:
        """The segment to drive now, or ``None`` at the end of the route."""
        if self._current is None:
            self._current = await anext(self._segments, None)
        return self._current

    def advance(self) -> None:
        """Move on, now that the current segment has been driven.

        Deliberately after the drive rather than before it: a cancelled drive then leaves the
        segment current, instead of being silently skipped.
        """
        self._current = None
