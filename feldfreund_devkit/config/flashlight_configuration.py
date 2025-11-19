from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class FlashlightConfiguration:
    """Configuration for the flashlight of the Feldfreund robot.

    Defaults:
        name: 'flashlight'
        on_expander: True
        front_pin: 12
        back_pin: 23
    """
    name: str = 'flashlight'
    on_expander: bool = True
    front_pin: int = 12
    back_pin: int = 23


@dataclass(slots=True, kw_only=True)
class FlashlightMosfetConfiguration:
    """Configuration for the mosfet based flashlight of the Feldfreund robot.

    Defaults:
        name: str = 'flashlight_mosfet'
        on_expander: True
        pin: 2
    """
    name: str = 'flashlight_mosfet'
    on_expander: bool = True
    pin: int = 2
