from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class EstopConfiguration:
    """Configuration for the estop of the Field Friend robot.

    Defaults:
        name: 'estop'
        pin_1: 34
        pin_2: 35
    """
    name: str = 'estop'
    pin_1: int = 34
    pin_2: int = 35

    @property
    def pins(self) -> dict[str, int]:
        return {
            '1': self.pin_1,
            '2': self.pin_2,
        }
