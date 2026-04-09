from dataclasses import dataclass

from rosys.driving import DriveParameters


@dataclass(kw_only=True)
class TracksConfiguration:
    """Base configuration for Feldfreund tracks."""
    name: str = 'wheels'
    is_left_reversed: bool = False
    is_right_reversed: bool = False
    width: float = 0.502


@dataclass(kw_only=True)
class ODriveTracksConfiguration(TracksConfiguration):
    """Configuration for ODrive-based tracks with four motors (front/back, left/right)."""
    left_back_can_address: int = 0x000
    right_back_can_address: int = 0x100
    left_front_can_address: int = 0x200
    right_front_can_address: int = 0x300
    odrive_version: int = 4
    tooth_count: int = 15
    pitch: float = 0.033
    motor_gear_ratio: float = 12.52
    has_temperature_sensor: bool = False

    @property
    def m_per_tick(self) -> float:
        return self.tooth_count * self.pitch / self.motor_gear_ratio


@dataclass(kw_only=True)
class InnotronicTracksConfiguration(TracksConfiguration):
    """Configuration for Innotronic-based tracks with two motors (left/right)."""
    left_can_address: int = 5
    right_can_address: int = 37
    m_per_rad: float = 0.08275


def create_drive_parameters(*, linear_speed_limit: float = 0.3,
                            angular_speed_limit: float = 0.3,
                            minimum_turning_radius: float = 0.01,
                            can_drive_backwards: bool = False,
                            hook_offset: float = 0.20,
                            carrot_distance: float = 0.15,
                            carrot_step_fraction: float = 0.01,
                            carrot_offset: float = 0.35,
                            hook_bending_factor: float = 0.25,
                            minimum_drive_distance: float = 0.005,
                            throttle_at_end_distance: float = 0.2,
                            throttle_at_end_min_speed: float = 0.08,
                            **kwargs) -> DriveParameters:
    return DriveParameters(linear_speed_limit=linear_speed_limit,
                           angular_speed_limit=angular_speed_limit,
                           minimum_turning_radius=minimum_turning_radius,
                           can_drive_backwards=can_drive_backwards,
                           hook_offset=hook_offset,
                           carrot_distance=carrot_distance,
                           carrot_offset=carrot_offset,
                           carrot_step_fraction=carrot_step_fraction,
                           hook_bending_factor=hook_bending_factor,
                           minimum_drive_distance=minimum_drive_distance,
                           throttle_at_end_distance=throttle_at_end_distance,
                           throttle_at_end_min_speed=throttle_at_end_min_speed,
                           **kwargs)
