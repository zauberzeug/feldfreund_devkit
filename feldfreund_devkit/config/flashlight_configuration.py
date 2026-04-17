from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class FlashlightConfiguration:
    """Configuration for the flashlight of the Feldfreund robot.

    The LEDC channels must not collide with other PwmOutputs on the same chip.

    Defaults:
        name: 'flashlight'
        on_expander: True
        front_pin: 12
        back_pin: 23
        ledc_timer: 0
        front_ledc_channel: 0
        back_ledc_channel: 1
        duty_cycle: 1.0
    """
    name: str = 'flashlight'
    on_expander: bool = True
    front_pin: int = 12
    back_pin: int = 23
    ledc_timer: int = 0
    front_ledc_channel: int = 0
    back_ledc_channel: int = 1
    duty_cycle: float = 1.0


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
