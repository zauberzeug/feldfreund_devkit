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

    def __post_init__(self) -> None:
        if not 0.0 <= self.duty_cycle <= 1.0:
            raise ValueError('duty_cycle must be between 0 and 1')
        if not 0 <= self.ledc_timer <= 3:
            raise ValueError('ledc_timer must be between 0 and 3')
        for channel in (self.front_ledc_channel, self.back_ledc_channel):
            if not 0 <= channel <= 15:
                raise ValueError('LEDC channel must be between 0 and 15')
        if self.front_ledc_channel == self.back_ledc_channel:
            raise ValueError('front_ledc_channel and back_ledc_channel must differ')


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
