"""The robot's lamps: one class per lamp, a group per kind, and the setup building both kinds."""
import abc
from collections.abc import AsyncGenerator, Iterable, Sequence
from contextlib import AsyncExitStack, asynccontextmanager

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import (
    HeadlightConfigurations,
    LightConfiguration,
    WorkLightConfigurations,
    WorkLightMosfetConfiguration,
)
from .safety import SafetyMixin

SHARED_LEDC_TIMER = 0
LEDC_CHANNELS_PER_CHIP = 8
"""The classic ESP32 the Lizard firmware targets offers eight LEDC channels per speed mode."""
LIZARD_FULL_DUTY = 256
"""Lizard's PwmOutput runs 8 bit resolution, so 255 still leaves one step of 256 dark and only 256 is fully on."""

WORK_LIGHT_TITLE = 'Work Lights'
HEADLIGHT_TITLE = 'Headlights'


class Light(rosys.hardware.Module, abc.ABC):
    """Base class for a single light with on/off and duty cycle control."""

    def __init__(self, *, name: str, safety_managed: bool = True, **kwargs) -> None:
        super().__init__(**kwargs)
        self.name = name
        self.safety_managed = safety_managed
        """Whether the robot's safety system force-disables this lamp on an e-stop or bumper trigger."""
        self._duty_cycle: float = 1.0
        self._is_active: bool = False
        self._users: int = 0

    @property
    def duty_cycle(self) -> float:
        return self._duty_cycle

    @property
    def is_active(self) -> bool:
        return self._is_active

    @asynccontextmanager
    async def on(self) -> AsyncGenerator[None, None]:
        """Keep the light on for the block; it stays on until the last concurrent block exits."""
        try:
            self._users += 1
            if self._users == 1:
                await self.turn_on()
            yield
        finally:
            self._users -= 1
            if self._users == 0:
                await self.turn_off()

    async def turn_on(self) -> None:
        self._is_active = True
        self.log.debug('Turning on light %s', self.name)

    async def turn_off(self) -> None:
        self._is_active = False
        self.log.debug('Turning off light %s', self.name)

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the light.

        :param duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        self._duty_cycle = duty_cycle
        self.log.debug('Setting duty cycle of %s to %s', self.name, duty_cycle)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label(self.name.replace('_', ' ').title()).classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)
            ui.slider(min=0, max=1, step=0.01, on_change=lambda e: self.set_duty_cycle(e.value)) \
                .bind_value_from(self, '_duty_cycle')


class Lights:
    """The lights of one robot, switchable as a group and individually by name."""

    def __init__(self, lights: Iterable[Light], *, title: str = 'Lights') -> None:
        """
        :param lights: the robot's lights
        :param title: the label this group shows in the developer UI
        :raises ValueError: if two lights share a name
        """
        self.title = title
        self.lights: dict[str, Light] = {}
        for light in lights:
            if light.name in self.lights:
                raise ValueError(f'duplicate light name "{light.name}"')
            self.lights[light.name] = light

    @property
    def is_active(self) -> bool:
        return any(light.is_active for light in self.lights.values())

    def __getitem__(self, name: str) -> Light:
        """Get a single light by name.

        :param name: the name the light was configured with
        :return: the light
        :raises KeyError: if the robot has no light of that name
        """
        return self.lights[name]

    @asynccontextmanager
    async def on(self) -> AsyncGenerator[None, None]:
        """Keep all lights on for the block; they stay on until the last concurrent block exits."""
        async with AsyncExitStack() as stack:
            for light in self.lights.values():
                await stack.enter_async_context(light.on())
            yield

    async def turn_on(self) -> None:
        for light in self.lights.values():
            await light.turn_on()

    async def turn_off(self) -> None:
        for light in self.lights.values():
            await light.turn_off()

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of every light.

        :param duty_cycle: float between 0 and 1
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        for light in self.lights.values():
            await light.set_duty_cycle(duty_cycle)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label(self.title).classes('text-center text-bold')
            with ui.button_group():
                ui.button('ALL ON', on_click=self.turn_on)
                ui.button('ALL OFF', on_click=self.turn_off)
            for light in self.lights.values():
                light.developer_ui()


class LightHardware(Light, rosys.hardware.ModuleHardware, SafetyMixin):
    """Light hardware implementation using a PWM output."""

    def __init__(self, config: LightConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None,
                 ledc_timer: int,
                 ledc_channel: int) -> None:
        """
        :param config: the lamp's configuration, whose name is the lamp's Lizard variable
        :param robot_brain: the robot brain driving the lamp
        :param expander: the port expander the lamp sits on, or None if it sits on the main chip
        :param ledc_timer: the LEDC timer the lamp's PWM output runs on
        :param ledc_channel: the LEDC channel the lamp occupies on its chip
        """
        self.config = config
        self.expander = expander
        chip = f'{expander.name}.' if expander is not None and config.on_expander else ''
        lizard_code = remove_indentation(f'''
            {config.name} = {chip}PwmOutput({config.pin}, {ledc_timer}, {ledc_channel})
            {config.name}.duty = {self._convert_duty_cycle_to_lizard(config.duty_cycle)}
        ''')
        if config.initially_on:
            lizard_code += f'{config.name}.on()\n'
        super().__init__(name=config.name, robot_brain=robot_brain, lizard_code=lizard_code,
                         safety_managed=config.safety_managed)
        self._duty_cycle = config.duty_cycle
        self._is_active = config.initially_on

    @property
    def enable_code(self) -> str:
        return f'{self.name}.enable();'

    @property
    def disable_code(self) -> str:
        return f'{self.name}.disable();'

    @staticmethod
    def _convert_duty_cycle_to_lizard(duty_cycle: float) -> int:
        """Convert the duty cycle to Lizard's PwmOutput duty, where 256 keeps the output on for the whole period.

        :param duty_cycle: float between 0 and 1
        :return: int between 0 and 256
        :raises ValueError: if duty cycle is not between 0 and 1
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        return int(duty_cycle * LIZARD_FULL_DUTY)

    async def turn_on(self) -> None:
        """Turn on the light if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning on light failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.name}.on()')

    async def turn_off(self) -> None:
        """Turn off the light if the robot brain is ready."""
        if not self.robot_brain.is_ready:
            self.log.error('Turning off light failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.name}.off()')

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set the duty cycle of the light if the robot brain is ready."""
        if not 0 <= duty_cycle <= 1:
            raise ValueError('duty cycle must be between 0 and 1')
        if not self.robot_brain.is_ready:
            self.log.error('Setting duty cycle failed. Robot Brain is not ready.')
            return
        await super().set_duty_cycle(duty_cycle)
        await self.robot_brain.send(f'{self.name}.duty={self._convert_duty_cycle_to_lizard(self.duty_cycle)};')


class WorkLightHardwareMosfet(Light, rosys.hardware.ModuleHardware, SafetyMixin):
    """Work light hardware implementation using a MOSFET switch.

    Deprecated: only used on U4. Not built into any new robot; do not extend.
    """

    def __init__(self, config: WorkLightMosfetConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        chip = f'{expander.name}.' if expander is not None and config.on_expander else ''
        # the duty cycle should ideally vary with battery voltage, but a fixed value is what U4 ships with
        lizard_code = remove_indentation(f'''
            {config.name} = {chip}PwmOutput({config.pin})
            {config.name}.duty = 204
        ''')
        super().__init__(name=config.name, robot_brain=robot_brain, lizard_code=lizard_code)
        self._duty_cycle: float = 204 / LIZARD_FULL_DUTY

    @property
    def enable_code(self) -> str:
        return f'{self.name}.enable();'

    @property
    def disable_code(self) -> str:
        return f'{self.name}.disable();'

    async def turn_on(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning on work light failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.name}.on()')

    async def turn_off(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning off work light failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.name}.off()')

    async def set_duty_cycle(self, duty_cycle: float) -> None:
        """Reject a brightness the fixed mosfet output cannot run.

        :param duty_cycle: ignored
        :raises RuntimeError: always
        """
        raise RuntimeError(f'{self.name} burns at a fixed brightness and cannot be dimmed')

    def developer_ui(self) -> None:
        with ui.column():
            ui.label(self.name.replace('_', ' ').title()).classes('text-center text-bold')
            with ui.button_group():
                ui.button('ON', on_click=self.turn_on)
                ui.button('OFF', on_click=self.turn_off)


class LightSimulation(Light, rosys.hardware.ModuleSimulation):
    """Simulated light for testing."""

    def __init__(self, config: LightConfiguration | WorkLightMosfetConfiguration, **kwargs) -> None:
        """
        :param config: the lamp's configuration, as the hardware lamp takes it
        """
        super().__init__(name=config.name, **kwargs)
        if isinstance(config, LightConfiguration):
            self._duty_cycle = config.duty_cycle
            self._is_active = config.initially_on


def light_modules(*groups: Lights | None) -> list[Light]:
    """The single lamps of the given groups, as the robot's module list and its safety system take them.

    :param groups: the robot's lamp groups, each None if it has none of that kind
    :return: every lamp of every group, in declaration order
    """
    return [light for group in groups if group is not None for light in group.lights.values()]


def setup_light_hardware(work_light_configs: WorkLightConfigurations,
                         headlight_configs: HeadlightConfigurations, *,
                         robot_brain: rosys.hardware.RobotBrain,
                         expander: rosys.hardware.ExpanderHardware) -> tuple[Lights | None, Lights | None]:
    """Build the work lights and headlights, assigning each lamp a free LEDC channel on its chip.

    Lamps share one LEDC timer; only the channel number must be unique per chip (`on_expander`).
    Channels are handed out by declaration order, work lights first, so nobody has to pick a
    channel number by hand or risk a silent collision with a neighbour. The deprecated mosfet
    work light takes no channel from this count; it must stay the only lamp on its chip.

    A lamp's name is its variable in Lizard's flat namespace, so it has to be free of every other
    module of the robot, not just of the other lamps: a lamp named 'left' would redeclare the drive
    motor of that name.

    :param work_light_configs: the robot's work light configurations, a single one, or none
    :param headlight_configs: the robot's headlight configurations, a single one, or none
    :param robot_brain: the robot brain driving the lamps
    :param expander: the port expander carrying the lamps configured to sit on it
    :return: the work lights and the headlights, each None if the robot has none of that kind
    :raises NotImplementedError: if a configuration type is unknown
    :raises ValueError: if two lamps share a name or a chip runs out of LEDC channels
    """
    work_configs = _as_light_configs(work_light_configs)
    headlight_light_configs = _as_light_configs(headlight_configs)
    _check_unique_names(work_configs, headlight_light_configs)
    next_channel: dict[bool, int] = {}
    groups: tuple[list[Light], list[Light]] = ([], [])
    for lights, configs in zip(groups, (work_configs, headlight_light_configs), strict=True):
        for config in configs:
            if isinstance(config, LightConfiguration):
                channel = next_channel.setdefault(config.on_expander, 0)
                if channel >= LEDC_CHANNELS_PER_CHIP:
                    raise ValueError(f'light "{config.name}" finds no free LEDC channel on its chip')
                next_channel[config.on_expander] = channel + 1
                lights.append(LightHardware(config, robot_brain,
                                            expander=expander if config.on_expander else None,
                                            ledc_timer=SHARED_LEDC_TIMER,
                                            ledc_channel=channel))
            elif isinstance(config, WorkLightMosfetConfiguration):
                lights.append(WorkLightHardwareMosfet(config, robot_brain,
                                                      expander=expander if config.on_expander else None))
            else:
                raise NotImplementedError(f'Unknown light configuration: {config}')
    return (Lights(groups[0], title=WORK_LIGHT_TITLE) if work_configs else None,
            Lights(groups[1], title=HEADLIGHT_TITLE) if headlight_light_configs else None)


def setup_light_simulation(work_light_configs: WorkLightConfigurations,
                           headlight_configs: HeadlightConfigurations) -> tuple[Lights | None, Lights | None]:
    """Build one simulated module per configured lamp.

    :param work_light_configs: the robot's work light configurations, a single one, or none
    :param headlight_configs: the robot's headlight configurations, a single one, or none
    :return: the work lights and the headlights, each None if the robot has none of that kind
    :raises ValueError: if two lamps share a name
    """
    _check_unique_names(_as_light_configs(work_light_configs), _as_light_configs(headlight_configs))
    return (_simulate_group(work_light_configs, title=WORK_LIGHT_TITLE),
            _simulate_group(headlight_configs, title=HEADLIGHT_TITLE))


def _check_unique_names(work_configs: Sequence[LightConfiguration | WorkLightMosfetConfiguration],
                        headlight_configs: Sequence[LightConfiguration | WorkLightMosfetConfiguration]) -> None:
    """Reject a work light and a headlight sharing a name, which the hardware and simulation paths must agree on.

    :raises ValueError: if two lamps across both groups share a name
    """
    names = [config.name for config in (*work_configs, *headlight_configs)]
    for name in names:
        if names.count(name) > 1:
            raise ValueError(f'duplicate light name "{name}"')


def _simulate_group(configs: WorkLightConfigurations | HeadlightConfigurations, *, title: str) -> Lights | None:
    light_configs = _as_light_configs(configs)
    if not light_configs:
        return None
    return Lights((LightSimulation(config) for config in light_configs), title=title)


def _as_light_configs(configs: WorkLightConfigurations) -> list[LightConfiguration | WorkLightMosfetConfiguration]:
    """Read a lamp field that may hold a single configuration, a sequence of them or nothing.

    :param configs: the value a robot configured for one of its lamp fields
    :return: the lamp configurations in declaration order
    """
    if not configs:
        return []
    if isinstance(configs, LightConfiguration | WorkLightMosfetConfiguration):
        return [configs]
    return list(configs)
