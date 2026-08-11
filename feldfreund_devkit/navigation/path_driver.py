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
    """A stop a tool is waiting for: what to come to rest over, and the handshake around it.

    Holds the target rather than a pose, because where the robot must stand for the tool to reach it
    can only be worked out against the segment that actually passes it -- which may not be the one
    being driven when the stop is asked for.
    """

    target: Point
    tool_offset_x: float
    reached: asyncio.Event = field(default_factory=asyncio.Event)
    released: asyncio.Event = field(default_factory=asyncio.Event)
    refused: bool = False
    """Set when the robot rolled past the target before it could stop; the waiter then skips it."""


class PathDriver:
    """Drives the segments a navigation hands out, at the slowest speed anyone is asking for.

    Wraps rosys' :class:`Driver`, which follows one immutable spline at a time, and owns everything
    about *how* a segment is driven -- speed, direction, where to come to rest -- so a navigation
    only has to say *which* path to take.

    A tool that has to work at a standstill asks for a stop through :meth:`stop_over` while the
    drive is in flight. Because the driver's spline cannot be changed underneath it, the running
    drive is aborted and re-issued as a piece that *ends* at the stop pose; the driver then plans
    the deceleration backwards from it on its own.

    :param driver: the low-level driver executing velocities
    """

    STOP_LOOKAHEAD: float = 1.0
    """How far (in metres) from the segment being driven a target may lie and still be stopped at.
    Only that segment is known, so a target further out cannot be checked against the route --
    accepting one would risk leaving a tool waiting for a place the robot never reaches."""

    def __init__(self, driver: Driver) -> None:
        self.driver = driver
        self._caps: list[float] = []
        self._segment: DriveSegment | None = None
        self._stop: _Stop | None = None

        self.SEGMENT_STARTED = Event[DriveSegment]()
        """driving a segment has begun (argument: ``DriveSegment``)"""

        self.SEGMENT_COMPLETED = Event[DriveSegment]()
        """a segment has been driven to its end (argument: ``DriveSegment``)"""

    @contextmanager
    def limit(self, speed: float) -> Iterator[None]:
        """Cap the driving speed for as long as the scope is held.

        Caps compose: the robot drives at the slowest of everything currently asked for, so a
        caller can only ever slow it down. A cap entered while a drive is in flight applies from
        the next piece onwards, not immediately.
        """
        self._caps.append(speed)
        try:
            yield
        finally:
            self._caps.remove(speed)

    def speed_limit(self, segment: DriveSegment) -> float:
        """The slowest speed the robot, the segment and the scoped caps allow.

        The robot's own configured limit is a ceiling: nothing a route or an operator asks for may
        drive it faster than it is built to go.
        """
        limits = [self.driver.parameters.linear_speed_limit, *self._caps]
        if segment.speed_limit is not None:
            limits.append(segment.speed_limit)
        return min(limits)

    @asynccontextmanager
    async def stop_over(self, target: Point, tool_offset_x: float) -> AsyncGenerator[None, None]:
        """Come to rest with the tool on ``target`` and hold there for the body of the scope.

        The stop stays pending until the robot reaches it, so a target beyond the end of the segment
        being driven is honoured once the segment containing it starts -- the tool simply waits
        longer. The running drive is aborted and re-issued as a piece ending at the stop pose; the
        abort is noticed within one driver tick, so a target closer than about a tick's worth of
        travel is reached with an abrupt stop rather than a planned ramp.

        :raises CannotStop: nothing is being driven, the target is behind the robot or behind the
            segment being driven, or it lies further than :attr:`STOP_LOOKAHEAD` from that segment.
            The robot keeps going; skip this target.
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
        assert self._stop is None, 'only one stop at a time is supported'
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
        """Drive the segment: its spline, at its speed, in its direction, resting at its end if it says so.

        Returns once the segment has been driven to its end, however many stops were held on the
        way. A stop pending somewhere further along the route is left pending. To drive only part of
        a segment, pass a copy carrying that piece as its spline:
        ``replace(segment, spline=part, stop_at_end=False)``.
        """
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
                    return  # the segment is done; a stop further along stays pending
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
        """Whether the tool already sits on ``target``, so working it needs no driving at all.

        Asked while the robot stands at a stop, to work everything within the tool's grasp before
        rolling on: once it moves, even a tick's worth of travel puts a target level with the tool
        out of :meth:`_is_ahead`'s tolerance and it is skipped for good.
        """
        ahead = self.driver.pose.relative_point(target).x - tool_offset_x
        return abs(ahead) < self.driver.parameters.minimum_drive_distance

    def _is_ahead(self, target: Point, tool_offset_x: float) -> bool:
        """Whether the robot can still bring its tool onto ``target`` by driving on.

        Asked in the robot's own frame, so it holds whichever segment the target belongs to. The
        tolerance keeps a target the tool is already over workable: the piece left to drive is then
        shorter than the driver bothers with, and it simply comes to rest where it is.
        """
        ahead = self.driver.pose.relative_point(target).x - tool_offset_x
        return ahead > -self.driver.parameters.minimum_drive_distance

    def _reach(self, spline: Spline, target: Point, tool_offset_x: float) -> ToolReach:
        """Whether ``spline`` brings the tool onto ``target``, allowing for a robot already on it.

        The one place the tolerance is supplied, so admitting a stop and resolving it later cannot
        disagree about what counts as reachable -- a stop admitted but never resolvable would hold
        the tool for the rest of the route.
        """
        return tool_reach(spline, target, tool_offset_x,
                          tolerance=self.driver.parameters.minimum_drive_distance)

    def _is_within_reach(self, spline: Spline, target: Point) -> bool:
        """Whether ``target`` is close enough to the segment being driven to be worth stopping for.

        Measured from the segment, so it covers both a target beyond its end and one far off to the
        side. Checked on the target itself rather than on the reduced stop pose, because the
        reduction clamps to the segment's own parameter range: a target metres beyond the end would
        otherwise come back as a pose just past it and be stopped at, nowhere near what was asked.
        """
        t = spline.closest_point(target.x, target.y)
        return spline.pose(t).point.distance(target) <= self.STOP_LOOKAHEAD

    def _remaining(self, segment: DriveSegment) -> Spline:
        """What is left of the segment, starting where the robot stands.

        Never the whole spline: the driver's carrot only moves forward from the start of what it is
        given, so a spline the robot is already partway along would send it backwards.
        """
        here = segment.spline.closest_point(self.driver.pose.x, self.driver.pose.y)
        return sub_spline(segment.spline, here, 1.0)
