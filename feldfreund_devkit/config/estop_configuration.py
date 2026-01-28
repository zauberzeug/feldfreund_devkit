from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class EstopConfiguration:
    """Configuration for the estop of the Feldfreund robot.

    Defaults:
        name: 'estop'
        pin_front: 34
        pin_back: 35
    """
    name: str = 'estop'
    pin_front: int = 34
    pin_back: int = 35

    @property
    def pins(self) -> dict[str, int]:
        return {
            'front': self.pin_front,
            'back': self.pin_back,
        }
