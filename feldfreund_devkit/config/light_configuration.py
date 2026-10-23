from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class LightConfiguration:
    """Configuration for a single light of the Feldfreund robot.

    The name is the lamp's variable in Lizard's flat namespace, so it must be free of every other
    module of the robot; prefix it with the kind of lamp, as the shipped configs do.

    A lamp normally goes dark with the robot's other safety-critical outputs on an e-stop or
    bumper trigger, and stays dark until it is switched on again: re-enabling the robot clears
    the disable but does not light the lamp. Set `safety_managed` to False to keep a lamp out
    of that force-off, e.g. for a headlight whose job is making a stopped robot visible rather
    than illuminating the ground for plant detection.

    A lamp lit from the start has to set `safety_managed=False`, because after an e-stop it
    would stay dark until someone switches it on again.

    Defaults:
        on_expander: True
        duty_cycle: 1.0
        initially_on: False
        safety_managed: True
    """
    name: str
    on_expander: bool = True
    pin: int
    duty_cycle: float = 1.0
    initially_on: bool = False
    safety_managed: bool = True

    def __post_init__(self) -> None:
        if not 0.0 <= self.duty_cycle <= 1.0:
            raise ValueError('duty_cycle must be between 0 and 1')
        if self.initially_on and self.safety_managed:
            raise ValueError(f'light "{self.name}" is initially_on, so it must set safety_managed=False')


@dataclass(slots=True, kw_only=True)
class WorkLightMosfetConfiguration:
    """Configuration for the mosfet based work light of the Feldfreund robot.

    Deprecated: only used on U4. Not built into any new robot; do not extend.

    Defaults:
        name: str = 'work_light_mosfet'
        on_expander: True
        pin: 2
    """
    name: str = 'work_light_mosfet'
    on_expander: bool = True
    pin: int = 2


WorkLightConfigurations = LightConfiguration | WorkLightMosfetConfiguration | Sequence[LightConfiguration] | None
"""What a robot may configure as its work lights: one lamp, several, or none."""

HeadlightConfigurations = LightConfiguration | Sequence[LightConfiguration] | None
"""What a robot may configure as its headlights: one lamp, several, or none."""
