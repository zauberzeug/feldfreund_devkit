from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class HeadlightsConfiguration:
    """Configuration for the headlights of the Feldfreund robot that are controlled via pwm.

    Defaults:
        name: 'headlights'
        on_expander: False
        left_pin: 5
        right_pin: 4
        ledc_timer: 1
        left_ledc_channel: 2
        right_ledc_channel: 3
        left_duty_cycle: 1.0
        right_duty_cycle: 1.0
    """
    name: str = 'headlights'
    on_expander: bool = False
    left_pin: int = 5
    right_pin: int = 4
    ledc_timer: int = 1
    left_ledc_channel: int = 2
    right_ledc_channel: int = 3
    left_duty_cycle: float = 1.0
    right_duty_cycle: float = 1.0
