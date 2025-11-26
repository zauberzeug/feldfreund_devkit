import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import FlashlightConfiguration, FlashlightMosfetConfiguration
from .safety import SafetyMixin


class Flashlight(rosys.hardware.Module, abc.ABC):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._duty_cycle: float = 1.0
        self._is_active: bool = False

    @property
    def duty_cycle(self) -> float:
        return self._duty_cycle

    @property
    def is_active(self) -> bool:
        return self._is_active

    async def turn_on(self) -> None:
        self._is_active = True
        self.log.debug('Turning on flashlight')

    async def turn_off(self) -> None:
        self._is_active = False
        self.log.debug('Turning off flashlight')

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the flashlight.

        :param duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        self._duty_cycle = duty_cycle
        self.log.debug('Setting duty cycle to %s', duty_cycle)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Flashlight').classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)
            ui.slider(min=0, max=1, step=0.01, on_change=lambda e: self.set_duty_cycle(e.value)) \
                .bind_value_from(self, '_duty_cycle')


class FlashlightHardware(Flashlight, rosys.hardware.ModuleHardware, SafetyMixin):
    def __init__(self, config: FlashlightConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {config.name}_front = {expander.name + "." if expander else ""}PwmOutput({config.front_pin})
            {config.name}_front.duty = 255
            {config.name}_back = {expander.name + "." if expander else ""}PwmOutput({config.back_pin})
            {config.name}_back.duty = 255
            {config.name}_front.shadow({config.name}_back)
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    @property
    def enable_code(self) -> str:
        return f'{self.config.name}_front.enable(); {self.config.name}_back.enable();'

    @property
    def disable_code(self) -> str:
        return f'{self.config.name}_front.disable(); {self.config.name}_back.disable();'

    def _convert_duty_cycle_to_8_bit(self, duty_cycle: float) -> int:
        """Convert the duty cycle to a 8 bit value (0-255).

        :param duty_cycle: float between 0 and 1
        :return: int between 0 and 255
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        return int(duty_cycle * 255)

    async def turn_on(self) -> None:
        """Turn on the flashlight if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning on flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.config.name}_front.on()')

    async def turn_off(self) -> None:
        """Turn off the flashlight if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning off flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.config.name}_front.off()')

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the flashlight if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Setting duty cycle failed. Robot Brain is not ready.')
            return
        await super().set_duty_cycle(duty_cycle)
        duty = self._convert_duty_cycle_to_8_bit(self.duty_cycle)
        await self.robot_brain.send(
            f'{self.config.name}_front.duty={duty};'
            f'{self.config.name}_back.duty={duty};'
        )


class FlashlightHardwareMosfet(Flashlight, rosys.hardware.ModuleHardware, SafetyMixin):
    UPDATE_INTERVAL = 5.0

    def __init__(self, config: FlashlightMosfetConfiguration,
                 robot_brain: rosys.hardware.RobotBrain,
                 bms: rosys.hardware.Bms, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        self.bms = bms
        lizard_code = remove_indentation(f'''
            {config.name} = {expander.name + "." if expander else ""}PwmOutput({config.pin})
            {config.name}.duty = 204
        ''')
        # NOTE: Electronically the duty cycle should be variable based on the battery voltage, but 204 has been tested extensively and works well.
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        self._last_update: float = 0
        self._duty_cycle: float = 0.1

    @property
    def enable_code(self) -> str:
        return f'{self.config.name}.enable();'

    @property
    def disable_code(self) -> str:
        return f'{self.config.name}.disable();'

    async def turn_on(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning on flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.config.name}.on()')

    async def turn_off(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning off flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.config.name}.off()')

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Flashlight').classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)


class FlashlightSimulation(Flashlight, rosys.hardware.ModuleSimulation):
    ...
