"""Routes and waiting helpers shared by the orchestrator and path-driver tests."""
import rosys
from rosys.geometry import Pose

from feldfreund_devkit.navigation import DriveSegment, StaticNavigation

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
