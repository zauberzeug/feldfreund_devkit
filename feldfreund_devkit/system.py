import logging
import os
from typing import Any

import psutil
import rosys
from nicegui import Event
from rosys.driving import Odometer
from rosys.geometry import GeoPoint, GeoReference

from .config import get_config
from .feldfreund import FeldfreundHardware, FeldfreundSimulation
from .hardware import TeltonikaRouter


class System(rosys.persistence.Persistable):
    """
    The System is the core class of a RoSys project to initialize all components of a robot or system.
    """

    def __init__(self, robot_id: str, *, use_acceleration: bool = False) -> None:
        super().__init__()
        self._log = logging.getLogger('feldfreund.system')
        self.robot_id = robot_id
        assert self.robot_id != 'unknown'
        self.config = get_config(self.robot_id)
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        if not rosys.hardware.SerialCommunication.is_possible():
            rosys.enter_simulation()

        self.GNSS_REFERENCE_CHANGED: Event[[]] = Event()
        self.feldfreund: FeldfreundHardware | FeldfreundSimulation
        self.teltonika_router: TeltonikaRouter | None = None
        if rosys.is_simulation():
            self.feldfreund = FeldfreundSimulation(self.config, use_acceleration=use_acceleration)
        else:
            self.feldfreund = FeldfreundHardware(self.config)
            self.teltonika_router = self._setup_teltonika_router()
            rosys.on_repeat(self.log_status, 60 * 5)
        self.odometer = Odometer(self.feldfreund.wheels)
        self.update_gnss_reference(reference=GeoReference(GeoPoint.from_degrees(51.983204032849706, 7.434321368936861)))

    def _setup_teltonika_router(self) -> TeltonikaRouter | None:
        if teltonika_password := os.environ.get('TELTONIKA_PASSWORD', None):
            return TeltonikaRouter('http://192.168.42.1/api', teltonika_password)
        return None

    def update_gnss_reference(self, *, reference: GeoReference | None = None) -> None:
        if reference is None:
            if self.feldfreund.gnss is None:
                self.log.warning('Not updating GNSS reference: GNSS not configured')
                return
            if self.feldfreund.gnss.last_measurement is None:
                self.log.warning('Not updating GNSS reference: No GNSS measurement received')
                return
            reference = GeoReference(origin=self.feldfreund.gnss.last_measurement.point,
                                     direction=self.feldfreund.gnss.last_measurement.heading)
        self.log.debug('Updating GNSS reference to %s', reference)
        GeoReference.update_current(reference)
        self.GNSS_REFERENCE_CHANGED.emit()
        self.request_backup()

    def restart(self) -> None:
        os.utime('main.py')

    def log_status(self):
        msg = f'cpu: {psutil.cpu_percent():.0f}%  '
        msg += f'mem: {psutil.virtual_memory().percent:.0f}% '
        msg += f'temp: {self.get_jetson_cpu_temperature():.1f}°C '
        msg += f'battery: {self.feldfreund.bms.state.short_string}'
        self.log.info(msg)
        bms_logger = logging.getLogger('feldfreund.bms')
        bms_logger.info('Battery: %s', self.feldfreund.bms.state.short_string)

    def get_jetson_cpu_temperature(self):
        with open('/sys/devices/virtual/thermal/thermal_zone0/temp', encoding='utf-8') as f:
            temp = f.read().strip()
        return float(temp) / 1000.0  # Convert from milli °C to °C

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        ...
