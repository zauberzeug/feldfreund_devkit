from abc import ABC, abstractmethod
from collections.abc import AsyncIterator

from nicegui import Event

from .drive_segment import DriveSegment


class Navigation(ABC):
    """Produces the route to drive, one segment at a time.

    A navigation says *where* to go, not how the driving is done -- except where a segment itself
    demands a speed or a stop, which it carries. Segments are pulled one at a time, so a navigation
    that plans from what it currently sees can plan each one at the moment it starts.
    """

    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, *, name: str) -> None:
        self.name = name
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT
        """Forward speed the user allows; segments may ask for less, never for more."""

    @abstractmethod
    def segments(self) -> AsyncIterator[DriveSegment]:
        """Yield the segments to drive, in order, until the route ends.

        Raise to refuse to start at all; yielding nothing means there was legitimately nothing to
        do. The consumer closes the iterator, so cleanup belongs in a ``finally``.
        """


class StaticNavigation(Navigation):
    """A navigation whose route is planned before the drive starts.

    The route is kept private: a caller that could splice into it would race with the segment being
    driven. A navigation that needs to change its mind while driving -- to dock, or to turn onto a
    row -- writes its own :meth:`Navigation.segments` and yields those segments where they belong.
    """

    def __init__(self, *, name: str) -> None:
        super().__init__(name=name)
        self._path: list[DriveSegment] = []

        self.PATH_CHANGED = Event[list[DriveSegment]]()
        """The segments still to drive have changed (argument: ``list[DriveSegment]``)."""

    @abstractmethod
    def generate_path(self) -> list[DriveSegment]:
        """Plan the whole route. Returning an empty list means there is nothing to drive."""

    async def segments(self) -> AsyncIterator[DriveSegment]:
        self._path = self.generate_path()
        self._announce()
        while self._path:
            yield self._path[0]
            self._path.pop(0)
            self._announce()

    def _announce(self) -> None:
        self.PATH_CHANGED.emit(list(self._path))
