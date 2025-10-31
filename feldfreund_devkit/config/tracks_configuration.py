from dataclasses import dataclass


@dataclass(kw_only=True)
class TracksConfiguration:
    """Configuration for the tracks of the Feldfreund robot.

    Defaults:
        is_left_reversed: False
        is_right_reversed: False
        left_back_can_address: 0x000
        right_back_can_address: 0x100
        left_front_can_address: 0x200
        right_front_can_address: 0x300
        odrive_version: 4
        width: 0.502
        tooth_count: 15
        pitch: 0.033
        motor_gear_ratio: 12.52
    """
    is_left_reversed: bool = False
    is_right_reversed: bool = False
    left_back_can_address: int = 0x000
    right_back_can_address: int = 0x100
    left_front_can_address: int = 0x200
    right_front_can_address: int = 0x300
    odrive_version: int = 4
    width: float = 0.502
    tooth_count: int = 15
    pitch: float = 0.033
    motor_gear_ratio: float = 12.52

    @property
    def m_per_tick(self) -> float:
        return self.tooth_count * self.pitch / self.motor_gear_ratio
