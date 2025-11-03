from dataclasses import dataclass


@dataclass(kw_only=True)
class RobotBrainConfiguration:
    """Configuration for the robot brain of the Field Friend robot.

    Defaults:
        enable_esp_on_startup: None
    """
    name: str
    flash_params: list[str]
    enable_esp_on_startup: bool = False
    use_espresso: bool = False
