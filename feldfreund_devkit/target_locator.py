import logging
from abc import ABC, abstractmethod

import rosys


class TargetLocator(rosys.persistence.Persistable, ABC):
    """The TargetLocator serves as the base class for all classes that generate navigation or implement targets.

    For example generating a 3D target pose from a given image or point cloud.
    """

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('devkit.target_locator')
        self._is_active = False

    @property
    def is_active(self) -> bool:
        return self._is_active

    @is_active.setter
    def is_active(self, active: bool) -> None:
        if active:
            self.resume()
        else:
            self.pause()

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
