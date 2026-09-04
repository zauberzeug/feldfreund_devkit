import rosys

from ..config import BumperConfiguration


class BumperHardware(rosys.hardware.BumperHardware):
    """Hardware bumper that builds its Lizard pin wiring from a `BumperConfiguration`."""

    def __init__(self, config: BumperConfiguration, robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None = None,
                 estop: rosys.hardware.EStop | None = None) -> None:
        self.config = config
        super().__init__(robot_brain=robot_brain,
                         expander=expander if config.on_expander else None,
                         name=config.name,
                         pins=config.pins,
                         estop=estop)


class BumperSimulation(rosys.hardware.BumperSimulation):
    """Simulated bumper carrying its `BumperConfiguration` for interface symmetry with `BumperHardware`."""

    def __init__(self, config: BumperConfiguration, **kwargs) -> None:
        self.config = config
        super().__init__(**kwargs)
