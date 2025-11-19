from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class CanConfiguration:
    """Configuration for the can of the Field Friend robot.

    Defaults:
        name: 'can'
        on_expander: False
        rx_pin: 32
        tx_pin: 33
        baud: 1_000_000
    """
    name: str = 'can'
    on_expander: bool = False
    rx_pin: int = 32
    tx_pin: int = 33
    baud: int = 1_000_000
