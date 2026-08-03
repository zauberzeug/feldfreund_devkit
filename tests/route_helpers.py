"""Routes and waiting helpers shared by the orchestrator and path-driver tests."""
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import rosys
from rosys.geometry import Pose

from feldfreund_devkit.navigation import DriveSegment, PathDriver, StaticNavigation, drive_and_work, no_work

TOOL_OFFSET = 0.1
"""How far ahead of the robot origin the pretend tool sits."""


class OneLegNavigation(StaticNavigation):
    """Two metres straight ahead, in one go."""

    def __init__(self) -> None:
        super().__init__(name='One Leg')

    def generate_path(self) -> list[DriveSegment]:
        return [DriveSegment.from_poses(Pose(), Pose(x=2.0))]


class RowTurnRowNavigation(StaticNavigation):
    """Two workable rows with a non-workable turn between them."""

    def __init__(self) -> None:
        super().__init__(name='Row Turn Row')

    def generate_path(self) -> list[DriveSegment]:
        return [
            DriveSegment.from_poses(Pose(), Pose(x=1.0), use_implement=True, stop_at_end=False),
            DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), use_implement=True),
            DriveSegment.from_poses(Pose(x=2.0), Pose(x=3.0)),
            DriveSegment.from_poses(Pose(x=3.0), Pose(x=4.0), use_implement=True),
        ]


async def until(condition) -> None:
    """Wait for ``condition`` from inside a running automation."""
    while not condition():
        await rosys.sleep(0.1)


class NoDetection:
    """The detection channel for tools that look for nothing; the scope does nothing."""

    @asynccontextmanager
    async def running(self) -> AsyncIterator[None]:
        yield


def route_run(devkit_system, navigation, *, work=no_work, detection=None):
    """A path driver and the run that drives ``navigation`` with it."""
    path_driver = PathDriver(devkit_system.driver, speed_limit=lambda: navigation.linear_speed_limit)
    return path_driver, drive_and_work(navigation, path_driver, devkit_system.robot_locator,
                                       detection=detection or NoDetection(), work=work)
