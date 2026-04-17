import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import HeadlightsConfiguration


class Headlights(rosys.hardware.Module, abc.ABC):
    """Base class for headlights modules with on/off control."""

    def __init__(self, config: HeadlightsConfiguration, **kwargs) -> None:
        super().__init__(**kwargs)
        self.config = config
        self._is_active: bool = False

    @property
    def is_active(self) -> bool:
        return self._is_active

    async def turn_on(self) -> None:
        self._is_active = True
        self.log.debug('Turning on headlights')

    async def turn_off(self) -> None:
        self._is_active = False
        self.log.debug('Turning off headlights')

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Headlights').classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)


class HeadlightsHardware(Headlights, rosys.hardware.ModuleHardware):
    """Headlights hardware implementation using digital outputs."""

    def __init__(self, config: HeadlightsConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {config.name}_left = {expander.name + "." if config.on_expander else ""}Output({config.left_pin})
            {config.name}_right = {expander.name + "." if config.on_expander else ""}Output({config.right_pin})
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


class HeadlightsSimulation(Headlights, rosys.hardware.ModuleSimulation):
    """Simulated headlights for testing."""
