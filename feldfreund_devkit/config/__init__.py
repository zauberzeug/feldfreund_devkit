import importlib
import importlib.util
from pathlib import Path

from .battery_configuration import BatteryControlConfiguration, BmsConfiguration
from .bluetooth_configuration import BluetoothConfiguration
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


def config_from_file(config_file: Path | str) -> FeldfreundConfiguration:
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f'No configuration file found at: {config_path}')
    module_name = f'config.{config_path.stem}'
    spec = importlib.util.spec_from_file_location(module_name, config_path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Could not load configuration from: {config_path}')
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)
    return config_module.config


def config_from_id(robot_id: str, *, config_dir: str = 'config') -> FeldfreundConfiguration:
    config_file = Path(config_dir) / f'{robot_id.lower()}.py'
    return config_from_file(config_file)


__all__ = [
    'BatteryControlConfiguration',
    'BluetoothConfiguration',
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
    'config_from_file',
    'config_from_id',
    'create_drive_parameters',
]
