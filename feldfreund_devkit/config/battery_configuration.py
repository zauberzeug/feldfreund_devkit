from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class BatteryControlConfiguration:
    """Configuration for the battery control of the Field Friend robot.

    Defaults:
        name: 'battery_control'
        on_expander: True
        reset_pin: 15
        status_pin: 13
    """
    name: str = 'battery_control'
    on_expander: bool = True
    reset_pin: int = 15
    status_pin: int = 13


@dataclass(slots=True, kw_only=True)
class BmsConfiguration:
    """Configuration for the bms of the Field Friend robot.

    Defaults:
        name: 'bms'
        on_expander: True
        rx_pin: 26
        tx_pin: 27
        baud: 9600
        num: 2
        battery_low_threshold: 15.0
    """
    name: str = 'bms'
    on_expander: bool = True
    rx_pin: int = 26
    tx_pin: int = 27
    baud: int = 9600
    num: int = 2
    battery_low_threshold: float = 15.0
