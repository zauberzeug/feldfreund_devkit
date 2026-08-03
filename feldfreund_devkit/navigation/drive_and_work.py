import logging
from collections.abc import AsyncIterator
from contextlib import aclosing

import rosys
from rosys.analysis import track
from rosys.driving.pose_provider import PoseProvider

from ..implement import ImplementException
from .drive_segment import DriveSegment
from .navigation import Navigation
from .path_driver import PathDriver
from .work_context import Detection, WorkContext, WorkFunction

log = logging.getLogger('feldfreund.drive_and_work')


@track
async def drive_and_work(navigation: Navigation, path_driver: PathDriver, pose_provider: PoseProvider, *,
                         detection: Detection, work: WorkFunction) -> None:
    """Drive a whole route, letting a tool work the stretches that are workable.

    Owns the run rather than the route, so the navigation stays a planner and the ``PathDriver``
    stays a driver. Returns once the route has ended and the robot has come to rest.

    The tool's work runs alongside the drive for as long as the route stays workable -- a *working
    stretch* -- and is cancelled when the first non-working segment comes up. A tool therefore never
    runs during a headland turn, and a tool that works while moving is never interrupted at a
    segment boundary in the middle of a row.

    :param navigation: produces the route
    :param path_driver: drives the segments, at the slowest speed anyone is asking for
    :param pose_provider: where the robot is, handed to the tool
    :param detection: controls when the robot looks for what it works on
    :param work: the tool's work loop; pass :func:`no_work` for a run that only drives
    """
    async def drive_stretch(route: '_Route') -> None:
        """Drive workable segments until the one after them is not."""
        while (segment := await route.current()) is not None and segment.use_implement:
            await path_driver.drive(segment)
            route.advance()

    async def work_until_cancelled() -> None:
        """Run the tool's work loop for one stretch, and refuse to let it end the stretch itself.

        Returning early would look to ``parallelize`` like the stretch being over and halt the drive
        mid-row, so it is reported rather than obeyed.
        """
        await work(WorkContext(motion=path_driver, pose=pose_provider, detection=detection))
        raise ImplementException('the work loop returned; it must run until the stretch ends')

    # NOTE: stopping the wheels happens here rather than in a branch of the drive, because that is
    # the one place cleanup may still await: on the error path `parallelize` closes its branches
    # with `GeneratorExit`, under which awaiting is illegal.
    try:
        async with aclosing(navigation.segments()) as segments:
            route = _Route(segments)
            while (segment := await route.current()) is not None:
                if segment.use_implement:
                    await rosys.automation.parallelize(
                        drive_stretch(route),
                        work_until_cancelled(),
                        return_when_first_completed=True,
                    )
                    continue
                await path_driver.drive(segment)
                route.advance()
    finally:
        await path_driver.driver.wheels.stop()


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
