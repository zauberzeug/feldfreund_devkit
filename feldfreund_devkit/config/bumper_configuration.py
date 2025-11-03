from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class BumperConfiguration:
    """Configuration for the bumper of the Field Friend robot.

    Defaults:
        name: 'bumper'
        on_expander: True
    """
    pin_front_top: int
    pin_front_bottom: int
    pin_back: int
    name: str = 'bumper'
    on_expander: bool = True

    @property
    def pins(self) -> dict[str, int]:
        return {
            'front_top': self.pin_front_top,
            'front_bottom': self.pin_front_bottom,
            'back': self.pin_back
        }
