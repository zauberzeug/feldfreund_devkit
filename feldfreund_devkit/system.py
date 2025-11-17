import logging
import os
from typing import Any

import rosys


class System(rosys.persistence.Persistable):
    """
    The System is the core class of a RoSys project to initialize all components of a robot or system.
    """

    def __init__(self, robot_id: str, *, use_acceleration: bool = False) -> None:
        super().__init__()
        self._log = logging.getLogger('feldfreund.system')

        # add your components here like:
        # self.wheels: Wheels = WheelsHardware()

    def restart(self) -> None:
        os.utime('main.py')

    def log_status(self) -> None:
        msg = '== System Status: '
        # msg += f'speed: {self.wheels.angular_target_speed} '
        msg += '=='
        self._log.info(msg)

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...
