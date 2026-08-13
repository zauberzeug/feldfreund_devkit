import asyncio
from collections.abc import AsyncGenerator, Iterator
from contextlib import asynccontextmanager, contextmanager
from dataclasses import dataclass, field

from nicegui import Event
from rosys.analysis import track
from rosys.driving import Driver, DrivingAbortedException
from rosys.geometry import Point, Spline

from .drive_segment import DriveSegment
from .utils import Reach, ToolReach, sub_spline, tool_reach


class CannotStop(Exception):
    """Raised when a requested stop cannot be made, so the caller should skip its target."""


@dataclass
class _Stop:
    """A stop a tool is waiting for."""

    target: Point
    tool_offset_x: float
    reached: asyncio.Event = field(default_factory=asyncio.Event)
    released: asyncio.Event = field(default_factory=asyncio.Event)
    refused: bool = False
    """The robot rolled past the target before it could stop."""


class PathDriver:
    """Drives the segments a navigation hands out, offering speed and stop controls."""

    STOP_LOOKAHEAD: float = 1.0
    """How far (m) from the segment being driven a target may lie and still be stopped at."""

    def __init__(self, driver: Driver) -> None:
        self.driver = driver
        self._caps: list[float] = []
        self._segment: DriveSegment | None = None
        self._stop: _Stop | None = None

        self.SEGMENT_STARTED = Event[DriveSegment]()
        self.SEGMENT_COMPLETED = Event[DriveSegment]()

    @contextmanager
    def limit(self, speed: float) -> Iterator[None]:
        """Cap the driving speed for as long as the scope is held; effective from the next piece driven."""
        self._caps.append(speed)
        try:
            yield
        finally:
            self._caps.remove(speed)

    def speed_limit(self, segment: DriveSegment) -> float:
        """The slowest speed the robot, the segment and the scoped caps allow."""
        limits = [self.driver.parameters.linear_speed_limit, *self._caps]
        if segment.speed_limit is not None:
            limits.append(segment.speed_limit)
        return min(limits)

    @asynccontextmanager
    async def stop_over(self, target: Point, tool_offset_x: float) -> AsyncGenerator[None, None]:
        """Come to rest with the tool on ``target`` and hold there for the body of the scope.

        :raises CannotStop: the target cannot be driven onto; the robot keeps going, skip it
        """
        segment = self._segment
        if segment is None:
            raise CannotStop('nothing is being driven')
        if not self._is_ahead(target, tool_offset_x):
            raise CannotStop(f'{target} is already behind the robot')
        if self._reach(segment.spline, target, tool_offset_x).where is Reach.BEHIND:
            raise CannotStop(f'{target} lies behind the segment being driven, which no later one reaches back to')
        if not self._is_within_reach(segment.spline, target):
            raise CannotStop(f'{target} is more than {self.STOP_LOOKAHEAD} m off the segment being driven')
        if self._stop is not None:
            # not CannotStop: callers absorb that, and the second holder would take over the slot unnoticed
            raise AssertionError('only one stop at a time is supported')
        stop = self._stop = _Stop(target, tool_offset_x)
        self.driver.abort()  # NOTE: only ever while driving; an armed flag would hit the next drive
        try:
            await stop.reached.wait()
            if stop.refused:
                raise CannotStop(f'{target} fell behind before the robot could come to rest on it')
            yield
        finally:
            self._stop = None
            stop.released.set()

    @track
    async def drive(self, segment: DriveSegment) -> None:
        """Drive ``segment`` to its end, honoring pending stop and stops submitted later."""

        self._segment = segment
        remaining = segment.spline
        self.SEGMENT_STARTED.emit(segment)
        try:
            while True:
                stop = self._stop
                stop_t: float | None = None
                if stop is not None:
                    reach = self._reach(remaining, stop.target, stop.tool_offset_x)
                    if reach.where is Reach.BEHIND:
                        # NOTE: rolled past while the abort took effect; let the tool go, do not wait
                        stop.refused = True
                        stop.reached.set()
                        self._stop = stop = None
                    elif reach.where is Reach.ON:
                        stop_t = reach.t
                piece = remaining if stop_t is None else sub_spline(remaining, 0.0, stop_t)
                try:
                    await self._drive(segment, piece,
                                      stop_at_end=stop_t is not None or segment.stop_at_end)
                except DrivingAbortedException:
                    remaining = self._remaining(segment)  # a stop was asked for, or released
                    continue
                if stop is None or stop_t is None:
                    self.SEGMENT_COMPLETED.emit(segment)
                    return
                stop.reached.set()
                await stop.released.wait()
                remaining = self._remaining(segment)
        finally:
            self._segment = None

    async def _drive(self, segment: DriveSegment, spline: Spline, *, stop_at_end: bool) -> None:
        with self.driver.parameters.set(linear_speed_limit=self.speed_limit(segment),
                                        can_drive_backwards=segment.backward):
            await self.driver.drive_spline(spline, flip_hook=segment.backward,
                                           throttle_at_end=stop_at_end, stop_at_end=stop_at_end)

    def is_reached(self, target: Point, tool_offset_x: float) -> bool:
        """Whether the tool already sits on ``target``, so working it needs no driving at all."""
        ahead = self.driver.pose.relative_point(target).x - tool_offset_x
        return abs(ahead) < self.driver.parameters.minimum_drive_distance

    def _is_ahead(self, target: Point, tool_offset_x: float) -> bool:
        """Whether the robot can still bring its tool onto ``target`` by driving on."""
        ahead = self.driver.pose.relative_point(target).x - tool_offset_x
        return ahead > -self.driver.parameters.minimum_drive_distance

    def _reach(self, spline: Spline, target: Point, tool_offset_x: float) -> ToolReach:
        """Whether ``spline`` brings the tool onto ``target``, allowing for a robot already on it."""
        return tool_reach(spline, target, tool_offset_x,
                          tolerance=self.driver.parameters.minimum_drive_distance)

    def _is_within_reach(self, spline: Spline, target: Point) -> bool:
        """Whether ``target`` is close enough to the segment being driven to be worth stopping for."""
        t = spline.closest_point(target.x, target.y)
        return spline.pose(t).point.distance(target) <= self.STOP_LOOKAHEAD

    def _remaining(self, segment: DriveSegment) -> Spline:
        """What is left of the segment, starting where the robot stands."""
        here = segment.spline.closest_point(self.driver.pose.x, self.driver.pose.y)
        return sub_spline(segment.spline, here, 1.0)
