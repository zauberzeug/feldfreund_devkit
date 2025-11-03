from abc import abstractmethod

import rosys
from rosys.geometry import Point3d

from ..config import ImplementConfiguration


class ImplementHardware:
    def __init__(self, config: ImplementConfiguration) -> None:
        self._config = config

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def offset(self) -> Point3d:
        return self._config.offset

    @property
    @abstractmethod
    def modules(self) -> list[rosys.hardware.Module]:
        ...

    @abstractmethod
    def can_reach(self, local_point: rosys.geometry.Point) -> bool:
        ...

    @abstractmethod
    async def stop(self) -> None:
        ...
