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
    """
    is_left_reversed: bool = False
    is_right_reversed: bool = False
    left_back_can_address: int = 0x000
    right_back_can_address: int = 0x100
    left_front_can_address: int = 0x200
    right_front_can_address: int = 0x300
    odrive_version: int = 4
