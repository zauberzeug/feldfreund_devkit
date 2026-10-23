from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from typing import Any, NoReturn

import pytest
import rosys
from rosys.geometry import Point3d

from feldfreund_devkit import Implement, ImplementException, WorkContext, activated, never
from feldfreund_devkit.config import ImplementConfiguration


async def test_the_tool_that_fails_to_activate_is_named() -> None:
    """With two tools, a failing activation reports which of them failed."""
    with pytest.raises(ImplementException, match='Gripper: not homed'):
        async with activated([Tool('Screw'), Tool('Gripper', fails=True)]):
            pass


async def test_the_tool_that_fails_while_working_is_named() -> None:
    """With two tools, a failure part way through a row reports which of them failed."""
    async with activated([Tool('Screw'), Tool('Gripper', work_fails=True)]) as tools:
        with pytest.raises(ImplementException, match='Gripper: lost its reference'):
            await tools[1].work(WorkContext(motion=None, locator=None))  # type: ignore[arg-type]


async def test_a_tool_that_cannot_be_stopped_does_not_keep_the_others_running(devkit_system) -> None:
    """The tools share one serial link, so a tool the link cannot reach must not swallow the e-stop."""
    unreachable = Tool('Gripper', stop_fails=True)
    reachable = Tool('Screw')
    devkit_system.feldfreund.add_implement(unreachable)
    devkit_system.feldfreund.add_implement(reachable)

    await devkit_system.feldfreund.stop()

    assert reachable.is_stopped


class Tool(Implement[None]):

    def __init__(self, name: str, *, fails: bool = False,
                 work_fails: bool = False, stop_fails: bool = False) -> None:
        super().__init__(ImplementConfiguration(lizard_name=name, display_name=name, work_radius=0.0))
        self.fails = fails
        self.work_fails = work_fails
        self.stop_fails = stop_fails
        self.is_stopped = False

    @property
    def modules(self) -> list[rosys.hardware.Module]:
        return []

    async def stop(self) -> None:
        if self.stop_fails:
            raise RuntimeError('the robot brain is not ready')
        self.is_stopped = True

    @asynccontextmanager
    async def activated(self) -> AsyncGenerator[None, None]:  # pylint: disable=invalid-overridden-method
        if self.fails:
            raise ImplementException('not homed')
        yield None

    async def work(self, ctx: WorkContext, context: None) -> NoReturn:
        if self.work_fails:
            raise ImplementException('lost its reference')
        await never()

    def can_reach(self, local_point: Point3d) -> bool:
        return True

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        pass
