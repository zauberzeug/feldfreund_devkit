import pytest
from conftest import FakeSecrets

from feldfreund_devkit import ImplementDummy
from feldfreund_devkit.config import RobotBrainConfiguration, config_from_id
from feldfreund_devkit.feldfreund import FeldfreundSimulation


class CountingImplement(ImplementDummy):

    def __init__(self) -> None:
        super().__init__()
        self.stop_calls = 0

    async def stop(self) -> None:
        self.stop_calls += 1


async def test_stop_stops_every_implement(rosys_integration) -> None:
    robot = FeldfreundSimulation(config_from_id('example', secrets=FakeSecrets()))
    first, second = CountingImplement(), CountingImplement()
    robot.add_implement(first)
    robot.add_implement(second)
    await robot.stop()
    assert first.stop_calls == 1
    assert second.stop_calls == 1


def test_robot_brain_rejects_unsupported_baud_rate() -> None:
    RobotBrainConfiguration(name='rb', serial_baud_rate=921600)
    with pytest.raises(ValueError):
        RobotBrainConfiguration(name='rb', serial_baud_rate=912600)  # type: ignore[arg-type]
