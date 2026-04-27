import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import HeadlightsConfiguration


class Headlights(rosys.hardware.Module, abc.ABC):
    """Base class for headlights modules with on/off and duty cycle control."""

    def __init__(self, config: HeadlightsConfiguration, **kwargs) -> None:
        super().__init__(**kwargs)
        self.config = config
        self._left_duty_cycle: float = self.config.left_duty_cycle
        self._right_duty_cycle: float = self.config.right_duty_cycle
        self._is_active: bool = False

    @property
    def left_duty_cycle(self) -> float:
        return self._left_duty_cycle

    @property
    def right_duty_cycle(self) -> float:
        return self._right_duty_cycle

    @property
    def is_active(self) -> bool:
        return self._is_active

    async def turn_on(self) -> None:
        self._is_active = True
        self.log.debug('Turning on headlights')

    async def turn_off(self) -> None:
        self._is_active = False
        self.log.debug('Turning off headlights')

    async def set_left_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the left headlight.

        :param duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        await self.set_duty_cycles(duty_cycle, self._right_duty_cycle)

    async def set_right_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the right headlight.

        :param duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        await self.set_duty_cycles(self._left_duty_cycle, duty_cycle)

    async def set_duty_cycles(self, left_duty_cycle: float, right_duty_cycle: float) -> None:
        """Set the duty cycles of the headlights.

        :param left_duty_cycle: float between 0 and 1
        :param right_duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= left_duty_cycle <= 1:
            raise ValueError('left duty cycle must be between 0 and 1')
        if not 0 <= right_duty_cycle <= 1:
            raise ValueError('right duty cycle must be between 0 and 1')
        self._left_duty_cycle = left_duty_cycle
        self._right_duty_cycle = right_duty_cycle
        self.log.debug('Setting left duty cycle to %s and right duty cycle to %s', left_duty_cycle, right_duty_cycle)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Headlights').classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)
            ui.slider(min=0, max=1, step=0.01, on_change=lambda e: self.set_left_duty_cycle(e.value)) \
                .bind_value_from(self, '_left_duty_cycle')
            ui.slider(min=0, max=1, step=0.01, on_change=lambda e: self.set_right_duty_cycle(e.value)) \
                .bind_value_from(self, '_right_duty_cycle')


class HeadlightsHardware(Headlights, rosys.hardware.ModuleHardware):
    """Headlights hardware implementation using PWM outputs."""

    def __init__(self, config: HeadlightsConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        prefix = f'{expander.name}.' if expander is not None and config.on_expander else ''
        lizard_code = remove_indentation(f'''
            {config.name}_left = {prefix}PwmOutput({config.left_pin}, {config.ledc_timer}, {config.left_ledc_channel})
            {config.name}_left.duty = {self._convert_duty_cycle_to_8_bit(config.left_duty_cycle)}
            {config.name}_right = {prefix}PwmOutput({config.right_pin}, {config.ledc_timer}, {config.right_ledc_channel})
            {config.name}_right.duty = {self._convert_duty_cycle_to_8_bit(config.right_duty_cycle)}
            {config.name}_left.on()
            {config.name}_right.on()
        ''')
        super().__init__(config, robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        """Turn on the headlights if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning on headlights failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.config.name}_left.on(); {self.config.name}_right.on()')

    async def turn_off(self) -> None:
        """Turn off the headlights if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning off headlights failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.config.name}_left.off(); {self.config.name}_right.off()')

    async def set_duty_cycles(self, left_duty_cycle: float, right_duty_cycle: float) -> None:
        """Set the duty cycle of the headlights if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Setting duty cycle failed. Robot Brain is not ready.')
            return
        await super().set_duty_cycles(left_duty_cycle, right_duty_cycle)
        left_duty = self._convert_duty_cycle_to_8_bit(self._left_duty_cycle)
        right_duty = self._convert_duty_cycle_to_8_bit(self._right_duty_cycle)
        await self.robot_brain.send(
            f'{self.config.name}_left.duty={left_duty};'
            f'{self.config.name}_right.duty={right_duty};'
        )

    @staticmethod
    def _convert_duty_cycle_to_8_bit(duty_cycle: float) -> int:
        """Convert the duty cycle to a 8 bit value (0-255).

        :param duty_cycle: float between 0 and 1
        :return: int between 0 and 255
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        return int(duty_cycle * 255)


class HeadlightsSimulation(Headlights, rosys.hardware.ModuleSimulation):
    """Simulated headlights for testing."""
