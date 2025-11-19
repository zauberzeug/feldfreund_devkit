import importlib

from .battery_configuration import BatteryControlConfiguration, BmsConfiguration
from .bumper_configuration import BumperConfiguration
from .camera_configuration import CameraConfiguration, CircleSightPositions, CropConfiguration
from .can_configuration import CanConfiguration
from .estop_configuration import EstopConfiguration
from .feldfreund_configuration import FeldfreundConfiguration
from .flashlight_configuration import FlashlightConfiguration, FlashlightMosfetConfiguration
from .gnss_configuration import GnssConfiguration
from .implement_configuration import ImplementConfiguration
from .imu_configuration import ImuConfiguration
from .robot_brain_configuration import RobotBrainConfiguration
from .tracks_configuration import TracksConfiguration, create_drive_parameters


def get_config(robot_name: str) -> FeldfreundConfiguration:
    try:
        module_name = f'config.{robot_name.lower()}'
        config_module = importlib.import_module(module_name)
        importlib.reload(config_module)  # reload to avoid cached config
        return config_module.config
    except ImportError as e:
        raise RuntimeError(f'No configuration found for robot: {robot_name}') from e


__all__ = [
    'BatteryControlConfiguration',
    'BmsConfiguration',
    'BumperConfiguration',
    'CameraConfiguration',
    'CanConfiguration',
    'CircleSightPositions',
    'CropConfiguration',
    'EstopConfiguration',
    'FeldfreundConfiguration',
    'FlashlightConfiguration',
    'FlashlightMosfetConfiguration',
    'GnssConfiguration',
    'ImplementConfiguration',
    'ImuConfiguration',
    'RobotBrainConfiguration',
    'TracksConfiguration',
    'create_drive_parameters',
    'get_config',
]
