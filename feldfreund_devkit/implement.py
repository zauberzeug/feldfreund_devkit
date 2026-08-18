from abc import abstractmethod
from collections.abc import AsyncGenerator
from contextlib import AbstractAsyncContextManager, asynccontextmanager
from typing import Any, Generic, NoReturn, TypeVar

import rosys
from rosys.geometry import Point3d, Pose3d

from .config import ImplementConfiguration
from .work_context import WorkContext, never

# NOTE: not a PEP 695 type parameter, which would need Python 3.12
ImplementContext = TypeVar('ImplementContext')


class ImplementException(Exception):
    """Raised when an implement operation fails."""


class Implement(rosys.persistence.Persistable, Generic[ImplementContext]):
    """Base class for robot implements like weeding tools or cameras."""

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
    def activated(self) -> AbstractAsyncContextManager[ImplementContext]:
        """Make the tool ready to work and hand back what it keeps for this run."""

    @abstractmethod
    async def work(self, ctx: WorkContext, context: ImplementContext) -> NoReturn:
        """Work until cancelled."""

    @abstractmethod
    def can_reach(self, local_point: Point3d) -> bool:
        ...

    @abstractmethod
    def backup_to_dict(self) -> dict[str, Any]:
        ...

    @abstractmethod
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

    # NOTE: the decorator makes this the context manager the abstract asks for; pylint sees only the async def
    @asynccontextmanager
    async def activated(self) -> AsyncGenerator[None, None]:  # pylint: disable=invalid-overridden-method
        yield None

    async def work(self, ctx: WorkContext, context: None) -> NoReturn:
        await never()

    def can_reach(self, local_point: Point3d) -> bool:
        return True

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...
