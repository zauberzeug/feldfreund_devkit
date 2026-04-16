import importlib
import importlib.util
from pathlib import Path

from .battery_configuration import BatteryControlConfiguration, BmsConfiguration
from .bluetooth_configuration import BluetoothConfiguration
from .bumper_configuration import BumperConfiguration
from .camera_configuration import (
    CameraConfiguration,
    CameraSlotConfig,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
    create_calibration,
)
from .can_configuration import CanConfiguration
from .estop_configuration import EstopConfiguration
from .feldfreund_configuration import FeldfreundConfiguration
from .flashlight_configuration import FlashlightConfiguration, FlashlightMosfetConfiguration
from .gnss_configuration import GnssConfiguration
from .implement_configuration import ImplementConfiguration
from .imu_configuration import ImuConfiguration
from .robot_brain_configuration import RobotBrainConfiguration
from .robot_footprint import RobotFootprint
from .secrets import Secrets
from .tracks_configuration import (
    InnotronicTracksConfiguration,
    ODriveTracksConfiguration,
    TracksConfiguration,
    create_drive_parameters,
)


def config_from_file(config_file: Path | str, *, secrets: Secrets) -> FeldfreundConfiguration:
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f'No configuration file found at: {config_path}')
    module_name = f'config.{config_path.stem}'
    spec = importlib.util.spec_from_file_location(module_name, config_path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Could not load configuration from: {config_path}')
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)
    return config_module.build_config(secrets)


def config_from_id(robot_id: str, *, secrets: Secrets, config_dir: str = 'config') -> FeldfreundConfiguration:
    config_file = Path(config_dir) / f'{robot_id.lower()}.py'
    return config_from_file(config_file, secrets=secrets)


__all__ = [
    'BatteryControlConfiguration',
    'BluetoothConfiguration',
    'BmsConfiguration',
    'BumperConfiguration',
    'CameraConfiguration',
    'CameraSlotConfig',
    'CanConfiguration',
    'EstopConfiguration',
    'FeldfreundConfiguration',
    'FlashlightConfiguration',
    'FlashlightMosfetConfiguration',
    'GnssConfiguration',
    'ImplementConfiguration',
    'ImuConfiguration',
    'InnotronicTracksConfiguration',
    'MjpegCameraConfig',
    'ODriveTracksConfiguration',
    'RobotBrainConfiguration',
    'RobotFootprint',
    'RtspCameraConfig',
    'Secrets',
    'TracksConfiguration',
    'UsbCameraConfig',
    'config_from_file',
    'config_from_id',
    'create_calibration',
    'create_drive_parameters',
]
