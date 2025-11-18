from abc import abstractmethod
from typing import Any

import rosys
from rosys.analysis import track
from rosys.geometry import Frame3d, Point, Pose3d

from .config import ImplementConfiguration


class ImplementException(Exception):
    pass


class Implement(rosys.persistence.Persistable):

    def __init__(self, config: ImplementConfiguration) -> None:
        super().__init__()
        self._config = config
        self._frame = self._config.offset.as_frame('implement') if self._config is not None else Frame3d.zero()

    @property
    def name(self) -> str:
        return self._config.display_name

    @property
    def frame(self) -> Frame3d:
        return self._frame

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

    @track
    async def activate(self) -> bool:
        """Activate and prepare the implement for use"""
        return True

    @track
    async def deactivate(self) -> None:
        """Deactivate the implement and clean up after use"""

    @track
    async def start_workflow(self) -> None:
        """Called after robot has stopped via observation to perform it's workflow on a specific point on the ground

        Returns True if the robot can drive forward, if the implement whishes to stay at the current location, return False
        """

    @track
    async def stop_workflow(self) -> None:
        """Called after workflow has been performed to stop the workflow"""

    @track
    async def get_target(self) -> Point | None:
        """Return the target position to drive to."""
        return None

    @abstractmethod
    def can_reach(self, local_point: rosys.geometry.Point) -> bool:
        ...

    @abstractmethod
    async def is_ready(self) -> bool:
        ...

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...

    def settings_ui(self) -> None:
        """Create UI for settings and configuration."""

    def developer_ui(self) -> None:
        """Create UI for developer tools."""


class ImplementDummy(Implement):
    def __init__(self) -> None:
        super().__init__(ImplementConfiguration(lizard_name='None', display_name='None', work_radius=0.0))

    @property
    def modules(self) -> list[rosys.hardware.Module]:
        return []

    async def stop(self) -> None:
        pass

    async def is_ready(self) -> bool:
        return True

    def can_reach(self, local_point: Point) -> bool:
        return True
