from dataclasses import dataclass, field

from rosys.geometry import Rotation


@dataclass(slots=True, kw_only=True)
class ImuConfiguration:
    """Configuration for the IMU of the Field Friend robot.

    Defaults:
        name: 'imu'
    """
    name: str = 'imu'
    offset_rotation: Rotation = field(default_factory=Rotation.zero)
    min_gyro_calibration: float = 1.0
