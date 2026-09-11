from dataclasses import dataclass

SUPPORTED_BAUD_RATES = (115200, 230400, 460800, 921600)


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
        heartbeat_interval: 0.5
        supported_lizard_versions: None
        serial_baud_rate: 115200

    ``supported_lizard_versions`` is a PEP 440 version specifier like ``'<0.14.0'`` restricting which
    Lizard versions can be downloaded and flashed. ``None`` allows all versions.
    """
    name: str
    enable_esp_on_startup: bool = False
    nand: bool = False
    swap_pins: bool = False
    heartbeat_interval: float = 0.5
    supported_lizard_versions: str | None = None
    serial_baud_rate: int = 115200
    """Baud rate of the core console: 115200, 230400, 460800 or 921600, the rates Lizard's ``core.set_baudrate``
    accepts. It must match the rate persisted on the Robot Brain."""

    def __post_init__(self) -> None:
        if self.serial_baud_rate not in SUPPORTED_BAUD_RATES:
            raise ValueError(
                f'unsupported serial_baud_rate {self.serial_baud_rate}, use one of {SUPPORTED_BAUD_RATES}')

    @property
    def flash_params(self) -> list[str]:
        params = []
        if self.nand:
            params.append('--nand')
        if self.swap_pins:
            params.append('--swap')
        return params
