from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class HeadlightsConfiguration:
    """Configuration for the headlights of the Feldfreund robot that are controlled via digital outputs.

    Defaults:
        name: 'headlights'
        on_expander: False
        left_pin: 5
        right_pin: 4
    """
    name: str = 'headlights'
    on_expander: bool = False
    left_pin: int = 5
    right_pin: int = 4
