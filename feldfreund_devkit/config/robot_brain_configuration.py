from dataclasses import dataclass


@dataclass(kw_only=True)
class RobotBrainConfiguration:
    """Configuration for the robot brain of the Feldfreund robot.

    Defaults:
        enable_esp_on_startup: False
        nand: False
        swap_pins: False
    """
    name: str
    enable_esp_on_startup: bool = False
    # https://github.com/zauberzeug/lizard/blob/main/espresso.py
    nand: bool = False
    swap_pins: bool = False

    @property
    def flash_params(self) -> list[str]:
        params = []
        if self.nand:
            params.append('--nand')
        if self.swap_pins:
            params.append('--swap_pins')
        return params
