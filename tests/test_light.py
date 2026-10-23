import re

import pytest
from rosys.hardware.communication import Communication

from feldfreund_devkit.config import (
    FeldfreundConfiguration,
    HeadlightConfigurations,
    LightConfiguration,
    ODriveTracksConfiguration,
    RobotBrainConfiguration,
    WorkLightConfigurations,
    WorkLightMosfetConfiguration,
)
from feldfreund_devkit.feldfreund import FeldfreundHardware, FeldfreundSimulation
from feldfreund_devkit.hardware.light import Lights, LightSimulation


async def test_a_light_stays_on_until_the_last_user_leaves() -> None:
    light = LightSimulation(LightConfiguration(name='light', pin=12))
    async with light.on():
        async with light.on():
            assert light.is_active
        assert light.is_active
    assert not light.is_active


async def test_a_switch_on_that_failed_leaves_no_user_behind() -> None:
    """After a switch-on that raised, the next user still gets light instead of a lamp counted as on."""
    light = UnreliableLight()
    with pytest.raises(RuntimeError):
        async with light.on():
            pass

    light.fails = False
    async with light.on():
        assert light.is_active


async def test_the_group_lights_every_lamp_until_the_last_user_leaves() -> None:
    lights = Lights([LightSimulation(LightConfiguration(name='front', pin=12)),
                     LightSimulation(LightConfiguration(name='back', pin=23))])
    async with lights.on():
        async with lights.on():
            assert all(light.is_active for light in lights.lights.values())
        assert all(light.is_active for light in lights.lights.values())
    assert not any(light.is_active for light in lights.lights.values())


async def test_a_single_lamp_switches_off_without_the_others() -> None:
    """Switching one lamp off leaves the rest lit, and the group still counts as active."""
    lights = Lights([LightSimulation(LightConfiguration(name='front', pin=12)),
                     LightSimulation(LightConfiguration(name='back', pin=23))])
    await lights.turn_on()

    await lights['front'].turn_off()

    assert not lights['front'].is_active
    assert lights['back'].is_active
    assert lights.is_active


async def test_two_lamps_of_the_same_name_are_rejected() -> None:
    with pytest.raises(ValueError, match='duplicate light name'):
        Lights([LightSimulation(LightConfiguration(name='front', pin=12)),
                LightSimulation(LightConfiguration(name='front', pin=23))])


def test_an_estop_darkens_the_front_lamps_too(rosys_integration: None) -> None:
    """Headlights land in the safety system's disable(), so they go out with the work lights."""
    config = _config(work_lights=[LightConfiguration(name='work_light_front', pin=12)],
                     headlights=LightConfiguration(name='headlight_left', pin=5))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert robot.headlights is not None
    assert list(robot.headlights.lights) == ['headlight_left']
    disable_code = robot.robot_brain.lizard_code.split('when core.last_message_age')[0]
    assert 'work_light_front.disable()' in disable_code
    assert 'headlight_left.disable()' in disable_code


def test_a_lamp_can_opt_out_of_the_safety_system(rosys_integration: None) -> None:
    """A lamp with `safety_managed=False` appears in neither the safety system's `disable()` nor its `enable()`."""
    config = _config(work_lights=[LightConfiguration(name='work_light_front', pin=12)],
                     headlights=LightConfiguration(name='headlight_left', pin=5, safety_managed=False))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    disable_code = robot.robot_brain.lizard_code.split('when core.last_message_age')[0]
    assert 'work_light_front.disable()' in disable_code
    assert 'headlight_left.disable()' not in disable_code
    assert 'headlight_left.enable()' not in disable_code


def test_every_lamp_on_a_chip_gets_a_channel_of_its_own(rosys_integration: None) -> None:
    """Work lights and headlights are numbered through one shared count per chip, in declaration order."""
    config = _config(work_lights=[LightConfiguration(name='work_light_front', pin=12),
                                  LightConfiguration(name='work_light_back', pin=23)],
                     headlights=LightConfiguration(name='headlight_left', pin=5))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert _ledc_channels(robot) == {'work_light_front': 0, 'work_light_back': 1, 'headlight_left': 2}


def test_a_lamp_on_the_other_chip_counts_channels_for_itself(rosys_integration: None) -> None:
    """The main ESP and the expander carry separate LEDC peripherals, so both start at channel 0."""
    config = _config(work_lights=[LightConfiguration(name='work_light_front', pin=12),
                                  LightConfiguration(name='work_light_back', pin=5, on_expander=False)])

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert _ledc_channels(robot) == {'work_light_front': 0, 'work_light_back': 0}


def test_a_lamp_can_be_lit_from_the_start(rosys_integration: None) -> None:
    """Only a lamp configured as initially on is switched on by the startup code and lit in the simulation."""
    config = _config(work_lights=[LightConfiguration(name='work_light_front', pin=12)],
                     headlights=LightConfiguration(name='headlight_left', pin=5,
                                                   initially_on=True, safety_managed=False))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert 'headlight_left.on()' in robot.robot_brain.lizard_code
    assert 'work_light_front.on()' not in robot.robot_brain.lizard_code
    assert robot.headlights is not None
    assert robot.headlights['headlight_left'].is_active

    simulation = FeldfreundSimulation(config)

    assert simulation.headlights is not None
    assert simulation.headlights['headlight_left'].is_active


def test_a_boot_lit_lamp_must_opt_out_of_the_safety_system() -> None:
    """A lamp lit from the start has to set safety_managed=False."""
    with pytest.raises(ValueError, match='must set safety_managed=False'):
        LightConfiguration(name='headlight_left', pin=5, initially_on=True)


def test_a_lamp_at_full_brightness_stays_on_for_the_whole_pwm_period(rosys_integration: None) -> None:
    """Lizard's PwmOutput keeps its pin high only at duty 256, so full brightness has to reach it as 256."""
    config = _config(work_lights=LightConfiguration(name='work_light_front', pin=12, duty_cycle=1.0))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert 'work_light_front.duty = 256' in robot.robot_brain.lizard_code


async def test_the_mosfet_work_light_cannot_be_dimmed(rosys_integration: None) -> None:
    """The mosfet lamp refuses a brightness change and keeps reporting the one it burns at."""
    config = _config(work_lights=WorkLightMosfetConfiguration())

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert robot.work_lights is not None
    light = robot.work_lights['work_light_mosfet']
    with pytest.raises(RuntimeError, match='cannot be dimmed'):
        await light.set_duty_cycle(0.5)
    assert light.duty_cycle == pytest.approx(204 / 256)


async def test_a_brightness_outside_the_range_is_rejected_without_the_robot_brain(rosys_integration: None) -> None:
    """A duty cycle beyond 0 to 1 reaches the caller as an error even while the robot brain is not ready."""
    config = _config(work_lights=LightConfiguration(name='work_light_front', pin=12))

    robot = FeldfreundHardware(config, communication=_NoCommunication())

    assert not robot.robot_brain.is_ready
    assert robot.work_lights is not None
    with pytest.raises(ValueError, match='duty cycle must be between 0 and 1'):
        await robot.work_lights['work_light_front'].set_duty_cycle(1.5)


def test_a_work_light_and_a_headlight_of_the_same_name_are_rejected(rosys_integration: None) -> None:
    """A name is a Lizard variable, so the two kinds of lamp share one namespace."""
    config = _config(work_lights=LightConfiguration(name='lamp', pin=12),
                     headlights=LightConfiguration(name='lamp', pin=5))

    with pytest.raises(ValueError, match='duplicate light name'):
        FeldfreundHardware(config, communication=_NoCommunication())


def test_a_work_light_and_a_headlight_of_the_same_name_are_rejected_in_simulation_too(rosys_integration: None) -> None:
    """The simulation path must reject the same configs the hardware path does, not boot with one lamp shadowing the other."""
    config = _config(work_lights=LightConfiguration(name='lamp', pin=12),
                     headlights=LightConfiguration(name='lamp', pin=5))

    with pytest.raises(ValueError, match='duplicate light name'):
        FeldfreundSimulation(config)


def _ledc_channels(robot: FeldfreundHardware) -> dict[str, int]:
    """Read back which LEDC channel each lamp was given from the generated Lizard code."""
    matches = re.findall(r'(\w+) = (?:\w+\.)?PwmOutput\(\d+, \d+, (\d+)\)', robot.robot_brain.lizard_code)
    return {name: int(channel) for name, channel in matches}


def _config(*,
            work_lights: WorkLightConfigurations = None,
            headlights: HeadlightConfigurations = None) -> FeldfreundConfiguration:
    """Build the smallest robot configuration that boots, carrying only the lamps a test cares about.

    :param work_lights: the work light configurations the test drives, or none
    :param headlights: the headlight configurations the test drives, or none
    :return: the robot configuration
    """
    return FeldfreundConfiguration(
        robot_id='test',
        robot_brain=RobotBrainConfiguration(name='test'),
        wheels=ODriveTracksConfiguration(),
        work_lights=work_lights,
        headlights=headlights,
    )


class _NoCommunication(Communication):
    """A no-op communication so the hardware robot can be built without any serial port."""

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        pass

    async def read(self) -> str | None:
        return None


class UnreliableLight(LightSimulation):

    def __init__(self) -> None:
        super().__init__(LightConfiguration(name='light', pin=12))
        self.fails = True

    async def turn_on(self) -> None:
        if self.fails:
            raise RuntimeError('the robot brain is not ready')
        await super().turn_on()
