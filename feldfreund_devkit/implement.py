from abc import abstractmethod
from collections.abc import AsyncGenerator, AsyncIterator
from contextlib import AbstractAsyncContextManager, asynccontextmanager
from typing import Any

import rosys
from rosys.geometry import Point, Pose3d

from .config import ImplementConfiguration
from .work_context import WorkContext, never


class ImplementException(Exception):
    """Raised when an implement operation fails."""


class Implement[C](rosys.persistence.Persistable):
    """Base class for robot implements like weeding tools or cameras.

    ``C`` is whatever the tool keeps for the length of one run: what it set up when it was readied
    and needs again while it works. Only :meth:`activated` produces one and only :meth:`work` takes
    one, so a tool cannot be set to work without having been readied first. A tool that keeps
    nothing uses ``None``.
    """

    def __init__(self, config: ImplementConfiguration) -> None:
        super().__init__()
        self._config = config

    @property
    def name(self) -> str:
        return self._config.display_name

    @property
    def offset(self) -> Pose3d:
        return self._config.offset

    @property
    @abstractmethod
    def modules(self) -> list[rosys.hardware.Module]:
        ...

    @abstractmethod
    async def stop(self) -> None:
        ...

    @abstractmethod
    def activated(self) -> AbstractAsyncContextManager[C]:
        """Make the tool ready to work and hand back what it keeps for this run.

        Held for the length of a run: leaving the scope puts the tool away again, so a tool cannot
        be readied without also being cleaned up.

        :raises ImplementException: the tool cannot be made ready
        """

    @abstractmethod
    async def work(self, ctx: WorkContext, context: C) -> None:
        """Do whatever this implement does, for as long as the robot is on workable ground.

        Started when a working stretch begins and cancelled when it ends, so it must not return on
        its own. A tool that acts continuously from activation has nothing to do here but wait.

        :param context: what :meth:`activated` set up for this run
        """

    @abstractmethod
    def can_reach(self, local_point: Point) -> bool:
        ...

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...

    def settings_ui(self) -> None:
        """Create UI for settings and configuration."""

    def developer_ui(self) -> None:
        """Create UI for developer tools."""


class ImplementDummy(Implement[None]):
    """A no-op implement for testing or when no implement is attached."""

    def __init__(self) -> None:
        super().__init__(ImplementConfiguration(lizard_name='None', display_name='None', work_radius=0.0))

    @property
    def modules(self) -> list[rosys.hardware.Module]:
        return []

    async def stop(self) -> None:
        pass

    @asynccontextmanager
    async def activated(self) -> AsyncGenerator[None, None]:
        """Nothing to ready, and nothing to keep."""
        yield None

    async def work(self, ctx: WorkContext, context: None) -> None:
        """Nothing to do: this implement exists so the robot can drive without one."""
        await never()

    def can_reach(self, local_point: Point) -> bool:
        return True
