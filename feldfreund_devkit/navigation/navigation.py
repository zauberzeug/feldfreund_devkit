from abc import ABC, abstractmethod
from collections.abc import AsyncGenerator

from nicegui import Event

from .drive_segment import DriveSegment
from .recorded_track import GnssRequirement


class NavigationRefused(Exception):
    """Raised when a route cannot be planned at all, so the run never starts."""


class Navigation(ABC):
    """Produces the route to drive, one segment at a time."""

    @property
    def gnss_requirement(self) -> GnssRequirement | None:
        """The positioning quality this route needs."""
        return None

    @abstractmethod
    def segments(self, speed_limit: float) -> AsyncGenerator[DriveSegment, None]:
        """Yield the segments to drive, in order, until the route ends.

        Raise to refuse to start at all; yielding nothing means there was legitimately nothing to do.

        :param speed_limit: the fastest the mission allows; segments may ask for less, never more
        """


class StaticNavigation(Navigation):
    """A navigation whose route is planned before the drive starts."""

    def __init__(self) -> None:
        super().__init__()
        self._path: list[DriveSegment] = []

        self.PATH_CHANGED = Event[list[DriveSegment]]()
        """The segments still to drive have changed."""

    @abstractmethod
    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        """Plan the whole route; an empty list means there is nothing to drive."""

    async def segments(self, speed_limit: float) -> AsyncGenerator[DriveSegment, None]:  # pylint: disable=invalid-overridden-method
        self._path = self.generate_path(speed_limit)
        self._announce()
        while self._path:
            yield self._path[0]
            self._path.pop(0)
            self._announce()

    def _announce(self) -> None:
        self.PATH_CHANGED.emit(list(self._path))
