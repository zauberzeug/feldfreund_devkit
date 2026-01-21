from dataclasses import dataclass, field

from rosys.geometry import Rotation


@dataclass(slots=True, kw_only=True)
class ImuConfiguration:
    """Configuration for the IMU of the Feldfreund robot.

    Defaults:
        name: 'imu'
        offset_rotation: Rotation.zero
        min_gyro_calibration: 1.0
    """
    name: str = 'imu'
    offset_rotation: Rotation = field(default_factory=Rotation.zero)
    min_gyro_calibration: float = 1.0
