from abc import ABC, abstractmethod

import rosys

from feldfreund_devkit.hardware.tracks import TracksHardware


class SafetyMixin(ABC):

    @property
    @abstractmethod
    def enable_code(self) -> str:
        ...

    @property
    @abstractmethod
    def disable_code(self) -> str:
        ...


class Safety(ABC):
    """The safety module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 bumper: rosys.hardware.Bumper | None = None,
                 modules: list[SafetyMixin] | None = None) -> None:
        self.wheels = wheels
        self.estop = estop
        self.bumper = bumper
        self.modules = modules or []

    def add_module(self, module: SafetyMixin) -> None:
        self.modules.append(module)


class SafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, **kwargs) -> None:
        Safety.__init__(self, **kwargs)
        self.estop_active = False
        lizard_code = self._generate_lizard_code()
        if self.bumper is not None:
            self.bumper.BUMPER_TRIGGERED.subscribe(self.bumper_safety_notifications)
        if self.estop is not None:
            self.estop.ESTOP_TRIGGERED.subscribe(self.estop_triggered_safety_notifications)
            self.estop.ESTOP_RELEASED.subscribe(self.estop_released_safety_notifications)
        rosys.hardware.ModuleHardware.__init__(self, robot_brain, lizard_code)

    def add_module(self, module: SafetyMixin) -> None:
        super().add_module(module)
        self.lizard_code = self._generate_lizard_code()

    def _generate_lizard_code(self) -> str:
        assert isinstance(self.wheels, TracksHardware | rosys.hardware.WheelsHardware)
        assert isinstance(self.estop, rosys.hardware.EStopHardware)
        assert isinstance(self.bumper, rosys.hardware.BumperHardware)
        lizard_code = 'bool disabled = false\n'
        lizard_code += f'let disable do disabled = true; {self.wheels.name}.disable();'
        for module in self.modules:
            lizard_code += module.disable_code
        lizard_code += 'end\n'

        lizard_code += f'let enable do disabled = false; {self.wheels.name}.enable();'
        for module in self.modules:
            lizard_code += module.enable_code
        lizard_code += 'end\n'

        if self.estop.pins:
            lizard_code += 'bool estop_active = false\n'
            enable_conditions = [f'estop_{name}.active == false' for name in self.estop.pins]
            disable_conditions = [f'estop_{name}.active == true' for name in self.estop.pins]
            lizard_code += f'when {" and ".join(enable_conditions)} then estop_active = false; end\n'
            lizard_code += f'when {" or ".join(disable_conditions)} then estop_active = true; end\n'
        if self.bumper is not None:
            lizard_code += 'bool bumper_active = false\n'
            enable_conditions = [f'bumper_{name}.active == false' for name in self.bumper.pins]
            disable_conditions = [f'bumper_{name}.active == true' for name in self.bumper.pins]
            lizard_code += f'when {" and ".join(enable_conditions)} then bumper_active = false; end\n'
            lizard_code += f'when {" or ".join(disable_conditions)} then bumper_active = true; end\n'
        if self.estop.pins:
            lizard_code += f'when estop_active == false and disabled == true {"and bumper_active == false" if self.bumper is not None else ""} then enable(); end\n'
            lizard_code += f'when estop_active {"or bumper_active" if self.bumper is not None else ""} then disable(); end\n'

        lizard_code += f'when core.last_message_age > 1000 then {self.wheels.name}.speed(0, 0); end\n'
        lizard_code += 'when core.last_message_age > 20000 then disable(); end\n'
        return lizard_code

    def bumper_safety_notifications(self, pin: str) -> None:
        if self.estop_active:
            return
        if pin == 'front_top':
            rosys.notify('Front top bumper triggered', 'warning')
        elif pin == 'front_bottom':
            rosys.notify('Front bottom bumper triggered', 'warning')
        elif pin == 'back':
            rosys.notify('Back bumper triggered', 'warning')

    def estop_triggered_safety_notifications(self, name: str) -> None:
        rosys.notify(f'E-Stop {name} triggered', 'warning')
        self.estop_active = True

    async def estop_released_safety_notifications(self, name: str) -> None:
        rosys.notify(f'E-Stop {name} released')
        await rosys.sleep(0.1)
        self.estop_active = False


class SafetySimulation(Safety, rosys.hardware.ModuleSimulation):
    ...
