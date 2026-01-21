from dataclasses import dataclass
from typing import Literal


@dataclass(kw_only=True)
class BluetoothConfiguration:
    """Configuration for the Lizard Bluetooth module.

    Defaults:
        name: 'bluetooth'
        pin_code: 'default'
    """
    name: str = 'bluetooth'
    pin_code: int | None | Literal['default'] = 'default'
