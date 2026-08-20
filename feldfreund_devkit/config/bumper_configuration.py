from dataclasses import dataclass


@dataclass(slots=True, kw_only=True)
class BumperConfiguration:
    """Configuration for the bumper of the Feldfreund robot.

    Defaults:
        name: 'bumper'
        on_expander: True
    """
    pin_front_top: int | None
    pin_front_bottom: int | None
    pin_back: int | None
    name: str = 'bumper'
    on_expander: bool = True

    @property
    def pins(self) -> dict[str, int]:
        pins = {
            'front_top': self.pin_front_top,
            'front_bottom': self.pin_front_bottom,
            'back': self.pin_back,
        }
        return {name: pin for name, pin in pins.items() if pin is not None}
