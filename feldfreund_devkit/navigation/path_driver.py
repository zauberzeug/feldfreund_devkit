from collections.abc import Callable, Iterator
from contextlib import contextmanager

from rosys.analysis import track
from rosys.driving import Driver

from .drive_segment import DriveSegment


class PathDriver:
    """Drives the segments a navigation hands out, at the slowest speed anyone is asking for.

    Wraps rosys' :class:`Driver`, which follows one immutable spline at a time, and owns everything
    about *how* a segment is driven -- speed, direction, where to come to rest -- so a navigation
    only has to say *which* path to take.

    :param driver: the low-level driver executing velocities
    :param speed_limit: the ambient limit, read live because it is a user setting
    """

    def __init__(self, driver: Driver, *, speed_limit: Callable[[], float]) -> None:
        self.driver = driver
        self._ambient_limit = speed_limit
        self._caps: list[float] = []

    @contextmanager
    def limit(self, speed: float) -> Iterator[None]:
        """Cap the driving speed for as long as the scope is held.

        Caps compose: the robot drives at the slowest of everything currently asked for, so a
        caller can only ever slow it down.
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

    @track
    async def drive(self, segment: DriveSegment) -> None:
        """Drive the segment: its spline, at its speed, in its direction, resting at its end if it says so.

        To drive only part of a segment -- the stretch up to a target, or a nudge forward -- pass a
        copy carrying that spline: ``replace(segment, spline=part, stop_at_end=False)``. The manner
        of travel then still comes from the segment the part belongs to.
        """
        with self.driver.parameters.set(linear_speed_limit=self.speed_limit(segment),
                                        can_drive_backwards=segment.backward):
            await self.driver.drive_spline(segment.spline, flip_hook=segment.backward,
                                           throttle_at_end=segment.stop_at_end,
                                           stop_at_end=segment.stop_at_end)
