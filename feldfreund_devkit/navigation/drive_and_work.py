from collections.abc import AsyncIterator
from contextlib import aclosing
from functools import partial

import rosys
from rosys.analysis import track

from ..implement import Implement, ImplementException
from ..robot_locator import RobotLocator
from ..work_context import WorkContext, WorkFunction
from .drive_segment import DriveSegment
from .navigation import Navigation
from .path_driver import PathDriver


@track
async def drive(navigation: Navigation, path_driver: PathDriver, *, speed_limit: float) -> None:
    """Drive a navigation to its end without working."""
    await _drive_navigation(navigation, path_driver, speed_limit=speed_limit, work=None)


@track
async def drive_and_work[ImplementContext](navigation: Navigation, path_driver: PathDriver,
                                           locator: RobotLocator, *, speed_limit: float,
                                           implement: Implement[ImplementContext],
                                           context: ImplementContext) -> None:
    """Drive a navigation to its end, letting a tool work the stretches that are workable."""
    work = partial(implement.work, context=context)
    await _drive_navigation(navigation, path_driver, speed_limit=speed_limit, work=work,
                            locator=locator)


async def _drive_navigation(navigation: Navigation, path_driver: PathDriver, *, speed_limit: float,
                            work: WorkFunction | None,
                            locator: RobotLocator | None = None) -> None:
    async def drive_stretch(stream: '_SegmentStream') -> None:
        while (segment := await stream.current()) is not None and segment.use_implement:
            await path_driver.drive(segment)
            stream.advance()

    async def work_until_cancelled() -> None:
        assert work is not None and locator is not None
        await work(WorkContext(motion=path_driver, locator=locator))
        raise ImplementException('the work loop returned; it must run until the stretch ends')

    try:
        async with aclosing(navigation.segments(speed_limit)) as segments:
            stream = _SegmentStream(segments)
            while (segment := await stream.current()) is not None:
                if segment.use_implement and work is not None:
                    await rosys.automation.parallelize(
                        work_until_cancelled(),
                        drive_stretch(stream),
                        return_when_first_completed=True,
                    )
                    continue
                await path_driver.drive(segment)
                stream.advance()
    finally:
        await path_driver.driver.wheels.stop()


class _SegmentStream:
    """The segments to drive, one at a time, with a one-segment lookahead."""

    def __init__(self, segments: AsyncIterator[DriveSegment]) -> None:
        self._segments = segments
        self._current: DriveSegment | None = None

    async def current(self) -> DriveSegment | None:
        """The segment to drive now, or ``None`` at the end of the path."""
        if self._current is None:
            self._current = await anext(self._segments, None)
        return self._current

    def advance(self) -> None:
        self._current = None
