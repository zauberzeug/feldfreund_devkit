import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import FlashlightConfiguration


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


class FlashlightHardware(Flashlight, rosys.hardware.ModuleHardware):
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


class FlashlightHardwareMosfet(Flashlight, rosys.hardware.ModuleHardware):
    # TODO: check https://github.com/zauberzeug/field_friend/commit/1e4fcfcdf5107a6912346a9b306dd0afb0af2673
    UPDATE_INTERVAL = 5.0

    def __init__(self, config: FlashlightConfiguration,
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
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        self._last_update: float = 0
        self._duty_cycle: float = 0.1
        #  pylint: disable=unreachable
        raise NotImplementedError('FlashlightHardwareMosfet needs to be tested before it can be used again.')
        rosys.on_repeat(self.set_duty_cycle, 60.0)

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

    # async def _set_duty_cycle(self) -> None:
    #     if rosys.time() > self. last_update + self.UPDATE_INTERVAL:

    #         # get the current voltage from the BMS
    #         voltage = self.bms.state.voltage
    #         if not voltage:
    #             self._duty_cycle = 0
    #             return

    #         self.duty_cycle = self._calculate_duty_cycle(voltage)
    #         # get a 8 bit value for the duty cycle (0-255) no negative values
    #         # TODO: line below was uncommented, what should we do with this code block?
    #         # duty = int(self.duty_cycle * 255)

    #         # await self.robot_brain.send(
    #         #     f'{self.name}.duty={duty};'
    #         # )

    def _calculate_duty_cycle(self, voltage: float) -> float:
        """Calculate the duty cycle of the flashlight based on the battery voltage.

        Using the formula: Duty Cycle = (20 W) / (V * (0.1864 * V - 3.4409))
        :param voltage: float voltage of the battery
        :return: float duty cycle between 0 and 1
        """
        current = 0.1864 * voltage - 3.4409
        power_at_full_duty = voltage * current
        duty_cycle = 20 / power_at_full_duty
        return min(max(duty_cycle, 0), 1)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Flashlight').classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)


class FlashlightSimulation(Flashlight, rosys.hardware.ModuleSimulation):
    ...
