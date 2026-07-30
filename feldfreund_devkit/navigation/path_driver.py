import asyncio
from collections.abc import AsyncIterator, Callable, Iterator
from contextlib import asynccontextmanager, contextmanager

from rosys.analysis import track
from rosys.driving import Driver, DrivingAbortedException
from rosys.geometry import Point, Pose, Spline

from .drive_segment import DriveSegment
from .utils import pose_with_tool_at, sub_spline


class CannotStop(Exception):
    """Raised when a requested stop cannot be made, so the caller should skip its target."""


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
    :param speed_limit: the ambient limit, read live because it is a user setting
    """

    def __init__(self, driver: Driver, *, speed_limit: Callable[[], float]) -> None:
        self.driver = driver
        self._ambient_limit = speed_limit
        self._caps: list[float] = []
        self._segment: DriveSegment | None = None
        self._stop_pose: Pose | None = None
        self._arrived = asyncio.Event()
        self._released = asyncio.Event()

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
        """The slowest speed the segment, the scoped caps and the user allow."""
        limits = [self._ambient_limit(), *self._caps]
        if segment.speed_limit is not None:
            limits.append(segment.speed_limit)
        return min(limits)

    @asynccontextmanager
    async def stop_over(self, target: Point, tool_offset_x: float) -> AsyncIterator[None]:
        """Come to rest with the tool on ``target`` and hold there for the body of the scope.

        The running drive is aborted and re-issued as a piece ending at the stop pose. The abort is
        noticed within one driver tick, so a target closer than about a tick's worth of travel is
        reached with an abrupt stop rather than a planned ramp -- ask for stops with some lead.

        :raises CannotStop: nothing is driving, or the target is not ahead on the segment being
            driven. The robot keeps going; skip this target.
        """
        segment = self._segment
        if segment is None:
            raise CannotStop('nothing is being driven')
        pose = pose_with_tool_at(segment.spline, target, tool_offset_x)
        if not self._is_ahead(segment.spline, pose):
            raise CannotStop(f'{target} is not ahead on the segment being driven')
        assert self._stop_pose is None, 'only one stop at a time is supported'
        self._stop_pose = pose
        self._arrived.clear()
        self._released.clear()
        self.driver.abort()  # NOTE: only ever while driving; an armed flag would hit the next drive
        try:
            await self._arrived.wait()
            yield
        finally:
            self._stop_pose = None
            self._released.set()

    @track
    async def drive(self, segment: DriveSegment) -> None:
        """Drive the segment: its spline, at its speed, in its direction, resting at its end if it says so.

        Returns once the segment has been driven to its end, however many stops were held on the
        way. To drive only part of a segment, pass a copy carrying that piece as its spline:
        ``replace(segment, spline=part, stop_at_end=False)``.
        """
        self._segment = segment
        remaining = segment.spline
        try:
            while True:
                stop_pose = self._stop_pose
                piece = self._up_to(remaining, stop_pose) if stop_pose is not None else remaining
                try:
                    await self._drive(segment, piece, stop_at_end=stop_pose is not None or segment.stop_at_end)
                except DrivingAbortedException:
                    remaining = self._remaining(segment)  # a stop was asked for, or released
                    continue
                if stop_pose is None:
                    return
                self._arrived.set()
                await self._released.wait()
                self._released.clear()
                self._arrived.clear()
                remaining = self._remaining(segment)
        finally:
            self._segment = None
            self._arrived.set()  # NOTE: nothing will arrive anymore; let a waiting tool proceed

    async def _drive(self, segment: DriveSegment, spline: Spline, *, stop_at_end: bool) -> None:
        with self.driver.parameters.set(linear_speed_limit=self.speed_limit(segment),
                                        can_drive_backwards=segment.backward):
            await self.driver.drive_spline(spline, flip_hook=segment.backward,
                                           throttle_at_end=stop_at_end, stop_at_end=stop_at_end)

    def _is_ahead(self, spline: Spline, pose: Pose) -> bool:
        """Whether ``pose`` can still be driven to: ahead of the robot and not past the spline's end."""
        here = spline.closest_point(self.driver.pose.x, self.driver.pose.y)
        stop = spline.closest_point(pose.x, pose.y)
        return here <= stop < 1.0

    def _up_to(self, spline: Spline, pose: Pose) -> Spline:
        return sub_spline(spline, 0.0, spline.closest_point(pose.x, pose.y))

    def _remaining(self, segment: DriveSegment) -> Spline:
        """What is left of the segment, starting where the robot stands.

        Never the whole spline: the driver's carrot only moves forward from the start of what it is
        given, so a spline the robot is already partway along would send it backwards.
        """
        here = segment.spline.closest_point(self.driver.pose.x, self.driver.pose.y)
        return sub_spline(segment.spline, here, 1.0)
