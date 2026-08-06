"""Routes and waiting helpers shared by the run-loop and path-driver tests."""
import rosys
from rosys.geometry import Pose

from feldfreund_devkit import ImplementDummy, WorkContext, WorkFunction
from feldfreund_devkit.navigation import DriveSegment, PathDriver, StaticNavigation, drive_and_work

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


class AheadOfTheRobotNavigation(StaticNavigation):
    """A workable row that starts a metre in front of where the robot stands."""

    def __init__(self) -> None:
        super().__init__(name='Ahead Of The Robot')

    def generate_path(self) -> list[DriveSegment]:
        return [DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), use_implement=True)]


async def until(condition) -> None:
    """Wait for ``condition`` from inside a running automation."""
    while not condition():
        await rosys.sleep(0.1)


class ToolDoing(ImplementDummy):
    """A tool that does whatever a test asks of it while a stretch is worked."""

    def __init__(self, work: WorkFunction) -> None:
        super().__init__()
        self._work = work

    async def work(self, ctx: WorkContext, context: None) -> None:
        await self._work(ctx)


def route_run(devkit_system, navigation, *, work: WorkFunction | None = None):
    """A path driver and the run that drives ``navigation`` with it.

    Without ``work`` the tool keeps still, which is all a test of the driving itself needs.
    """
    path_driver = PathDriver(devkit_system.driver, speed_limit=lambda: navigation.linear_speed_limit)
    implement = ImplementDummy() if work is None else ToolDoing(work)
    return path_driver, drive_and_work(navigation, path_driver, devkit_system.robot_locator,
                                       implement=implement, context=None)
