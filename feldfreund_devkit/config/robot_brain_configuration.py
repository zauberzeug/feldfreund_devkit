from dataclasses import dataclass


@dataclass(kw_only=True)
class RobotBrainConfiguration:
    """Configuration for the robot brain of the Feldfreund robot.

    There are different versions of the Robot Brain's electronics.
    Make sure to configure the correct parameters for your Robot Brain.

    Also have a look the documentation of the espresso.py script.
    https://github.com/zauberzeug/lizard/blob/main/espresso.py

    Defaults:
        enable_esp_on_startup: False
        nand: False
        swap_pins: False
    """
    name: str
    enable_esp_on_startup: bool = False
    nand: bool = False
    swap_pins: bool = False

    @property
    def flash_params(self) -> list[str]:
        params = []
        if self.nand:
            params.append('--nand')
        if self.swap_pins:
            params.append('--swap')
        return params
