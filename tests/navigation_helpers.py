"""Navigations and waiting helpers shared by the run-loop and path-driver tests."""
import rosys
from conftest import DRIVE_SPEED
from rosys.geometry import Pose

from feldfreund_devkit import ImplementDummy, WorkContext, WorkFunction
from feldfreund_devkit.navigation import DriveSegment, PathDriver, StaticNavigation, drive_and_work

TOOL_OFFSET = 0.1
"""How far ahead of the robot origin the pretend tool sits."""


class OneLegNavigation(StaticNavigation):

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        return [DriveSegment.from_poses(Pose(), Pose(x=2.0), speed_limit=speed_limit)]


class RowTurnRowNavigation(StaticNavigation):

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        return [
            DriveSegment.from_poses(Pose(), Pose(x=1.0), use_implement=True, stop_at_end=False, speed_limit=speed_limit),
            DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), use_implement=True, speed_limit=speed_limit),
            DriveSegment.from_poses(Pose(x=2.0), Pose(x=3.0), speed_limit=speed_limit),
            DriveSegment.from_poses(Pose(x=3.0), Pose(x=4.0), use_implement=True, speed_limit=speed_limit),
        ]


class AheadOfTheRobotNavigation(StaticNavigation):

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        return [DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), use_implement=True, speed_limit=speed_limit)]


async def until(condition) -> None:
    """Wait for ``condition`` from inside a running automation."""
    while not condition():
        await rosys.sleep(0.1)


class ToolDoing(ImplementDummy):

    def __init__(self, work: WorkFunction) -> None:
        super().__init__()
        self._work = work

    async def work(self, ctx: WorkContext, context: None) -> None:
        await self._work(ctx)


def navigation_run(devkit_system, navigation, *, work: WorkFunction | None = None):
    """A path driver and the run that drives ``navigation`` with it; without ``work`` the tool keeps still."""
    path_driver = PathDriver(devkit_system.driver)
    implement = ImplementDummy() if work is None else ToolDoing(work)
    return path_driver, drive_and_work(navigation, path_driver, devkit_system.robot_locator,
                                       speed_limit=DRIVE_SPEED,
                                       implement=implement, context=None)
