from dataclasses import dataclass, field

from rosys.driving import DriveParameters

from .battery_configuration import BatteryControlConfiguration, BmsConfiguration
from .bumper_configuration import BumperConfiguration
from .camera_configuration import CameraConfiguration, CircleSightPositions
from .can_configuration import CanConfiguration
from .estop_configuration import EstopConfiguration
from .flashlight_configuration import FlashlightConfiguration, FlashlightMosfetConfiguration
from .gnss_configuration import GnssConfiguration
from .implement_configuration import ImplementConfiguration
from .imu_configuration import ImuConfiguration
from .robot_brain_configuration import RobotBrainConfiguration
from .tracks_configuration import TracksConfiguration, create_drive_parameters


@dataclass(kw_only=True)
class FeldfreundConfiguration:
    name: str
    battery_control: BatteryControlConfiguration = field(default_factory=BatteryControlConfiguration)
    bms: BmsConfiguration = field(default_factory=BmsConfiguration)
    bumper: BumperConfiguration | None = None
    can: CanConfiguration = field(default_factory=CanConfiguration)
    camera: CameraConfiguration | None = None
    circle_sight_positions: CircleSightPositions | None
    driver: DriveParameters = field(default_factory=create_drive_parameters)
    estop: EstopConfiguration = field(default_factory=EstopConfiguration)
    flashlight: FlashlightConfiguration | FlashlightMosfetConfiguration | None = None
    gnss: GnssConfiguration | None = None
    implement: ImplementConfiguration | None = None
    imu: ImuConfiguration | None = None
    robot_brain: RobotBrainConfiguration
    wheels: TracksConfiguration
