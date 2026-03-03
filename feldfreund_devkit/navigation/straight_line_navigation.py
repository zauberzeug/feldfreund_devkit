from typing import Any

from nicegui import ui
from rosys.geometry import Pose

from .waypoint_navigation import DriveSegment, WaypointNavigation


class StraightLineNavigation(WaypointNavigation):
    """Navigation that drives a straight line for a given length."""
    LENGTH: float = 2.0

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs, name='Straight Line')
        self.length = self.LENGTH
        self.backward = False

    def generate_path(self) -> list[DriveSegment]:
        last_pose = self.pose_provider.pose
        x = -self.length if self.backward else self.length
        target_pose = last_pose.transform_pose(Pose(x=x))
        segment = DriveSegment.from_poses(last_pose,
                                          target_pose,
                                          use_implement=not self.backward,
                                          backward=self.backward)
        return [segment]

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
        return super().backup_to_dict() | {
            'length': self.length,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.length = data.get('length', self.length)
