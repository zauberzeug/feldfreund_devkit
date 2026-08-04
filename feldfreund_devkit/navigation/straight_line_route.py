from typing import Any

from nicegui import ui
from rosys.driving.pose_provider import PoseProvider
from rosys.geometry import Pose

from .drive_segment import DriveSegment
from .navigation import StaticNavigation


class StraightLineRoute(StaticNavigation):
    """A straight stretch ahead of wherever the robot currently stands.

    Worked when driving forward; driving backward is for repositioning, so the tool stays off.
    """

    LENGTH: float = 2.0

    def __init__(self, pose_provider: PoseProvider) -> None:
        super().__init__(name='Straight Line')
        self.length = self.LENGTH
        self.backward = False
        self._pose_provider = pose_provider

    def generate_path(self) -> list[DriveSegment]:
        start = self._pose_provider.pose
        end = start.transform_pose(Pose(x=-self.length if self.backward else self.length))
        return [DriveSegment.from_poses(start, end, use_implement=not self.backward, backward=self.backward)]

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Length', step=0.5, min=0.05, format='%.1f', suffix='m', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'length') \
            .tooltip('Length to drive in meters')
        ui.checkbox('Backward') \
            .bind_value(self, 'backward') \
            .tooltip('The robot will drive backwards if enabled')

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {'length': self.length}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.length = data.get('length', self.length)
