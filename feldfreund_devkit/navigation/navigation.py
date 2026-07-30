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

    The remaining route stays visible in :attr:`path` for the 3D view, and stays *editable*: a
    navigation may splice segments in while driving -- to dock, or to turn onto a row -- and the
    next pull picks them up.
    """

    def __init__(self, *, name: str) -> None:
        super().__init__(name=name)
        self.path: list[DriveSegment] = []
        """The segments not driven yet, the first one being the one currently driven."""

        self.PATH_CHANGED = Event[list[DriveSegment]]()
        """The remaining route has changed (argument: ``list[DriveSegment]``)."""

    @abstractmethod
    def generate_path(self) -> list[DriveSegment]:
        """Plan the whole route. Returning an empty list means there is nothing to drive."""

    async def segments(self) -> AsyncIterator[DriveSegment]:
        self.path = self.generate_path()
        self.PATH_CHANGED.emit(self.path)
        while self.path:
            segment = self.path[0]
            yield segment
            if self.path and self.path[0] is segment:  # NOTE: a splice may have replaced the head
                self.path.pop(0)
                self.PATH_CHANGED.emit(self.path)
