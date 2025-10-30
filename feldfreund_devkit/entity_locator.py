from __future__ import annotations

import logging
from abc import ABC, abstractmethod

import rosys


class EntityLocator(rosys.persistence.Persistable, ABC):
    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('devkit.entity_locator')
        self._is_active = False

    @property
    def is_active(self) -> bool:
        return self._is_active

    def pause(self) -> None:
        if not self._is_active:
            return
        self.log.debug('pausing locator')
        self._is_active = False

    def resume(self) -> None:
        if self._is_active:
            return
        self.log.debug('resuming locator')
        self._is_active = True

    @abstractmethod
    def developer_ui(self) -> None:
        pass
