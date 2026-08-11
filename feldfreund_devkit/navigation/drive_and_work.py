import logging
from collections.abc import AsyncIterator
from contextlib import aclosing
from functools import partial
from typing import TypeVar

import rosys
from rosys.analysis import track
from rosys.driving.pose_provider import PoseProvider

from ..implement import Implement, ImplementException
from ..work_context import WorkContext, WorkFunction
from .drive_segment import DriveSegment
from .navigation import Navigation
from .path_driver import PathDriver

log = logging.getLogger('feldfreund.drive_and_work')

C = TypeVar('C')


@track
async def drive(navigation: Navigation, path_driver: PathDriver, *, speed_limit: float) -> None:
    """Drive a whole route without working."""
    await _drive_route(navigation, path_driver, speed_limit=speed_limit, work=None)


@track
async def drive_and_work(navigation: Navigation, path_driver: PathDriver,
                         pose_provider: PoseProvider, *, speed_limit: float,
                         implement: Implement[C], context: C) -> None:
    """Drive a whole route, letting a tool work the stretches that are workable.

    The tool's work runs alongside the drive for as long as the route stays workable -- a *working
    stretch* -- and is cancelled when the first non-working segment comes up. A tool therefore never
    runs during a headland turn, and is never interrupted at a segment boundary in the middle of a row.

    :param speed_limit: the fastest the mission allows; the route puts it on its segments
    :param context: what ``implement.activated`` kept for this run
    """
    work = partial(implement.work, context=context)
    await _drive_route(navigation, path_driver, speed_limit=speed_limit, work=work,
                       pose_provider=pose_provider)


async def _drive_route(navigation: Navigation, path_driver: PathDriver, *, speed_limit: float,
                       work: WorkFunction | None,
                       pose_provider: PoseProvider | None = None) -> None:
    async def drive_stretch(route: '_Route') -> None:
        while (segment := await route.current()) is not None and segment.use_implement:
            await path_driver.drive(segment)
            route.advance()

    async def work_until_cancelled() -> None:
        """Run the tool's work loop; a return would look like the stretch ending and halt the drive."""
        assert work is not None and pose_provider is not None
        await work(WorkContext(motion=path_driver, pose=pose_provider))
        raise ImplementException('the work loop returned; it must run until the stretch ends')

    # NOTE: the wheels are stopped here because it is the one place cleanup may still await: on the
    # error path `parallelize` closes its branches with `GeneratorExit`, under which awaiting is illegal

    try:
        async with aclosing(navigation.segments(speed_limit)) as segments:
            route = _Route(segments)
            while (segment := await route.current()) is not None:
                if segment.use_implement and work is not None:
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
    """The segments to drive, one at a time, with a one-segment lookahead.

    A stretch of workable segments ends at the first one that is not, which can only be found by
    pulling it -- so the run loop must be able to look at a segment before committing to drive it.
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
        """Move on to the next segment; called after a drive, so a cancelled one is not skipped."""
        self._current = None
