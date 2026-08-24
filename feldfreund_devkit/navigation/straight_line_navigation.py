from typing import Any

import rosys
from nicegui import ui
from rosys.driving.pose_provider import PoseProvider
from rosys.geometry import Pose

from ..settings_ui import SettingsUI
from .drive_segment import DriveSegment
from .navigation import StaticNavigation


class StraightLineNavigation(StaticNavigation, SettingsUI, rosys.persistence.Persistable):
    """Navigation that drives a straight line for a given length."""
    LENGTH: float = 2.0

    def __init__(self, pose_provider: PoseProvider) -> None:
        super().__init__()
        self.length = self.LENGTH
        self.backward = False
        self._pose_provider = pose_provider

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        start = self._pose_provider.pose
        end = start.transform_pose(Pose(x=-self.length if self.backward else self.length))
        return [DriveSegment.from_poses(start, end, use_implement=not self.backward,
                                        backward=self.backward, speed_limit=speed_limit)]

    def settings_ui(self) -> None:
        ui.number('Length', step=0.5, min=0.05, format='%.1f', suffix='m', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'length') \
            .tooltip('Length to drive in meters')
        ui.checkbox('Backward') \
            .bind_value(self, 'backward') \
            .tooltip('The robot will drive backwards if enabled')

    def backup_to_dict(self) -> dict[str, Any]:
        return {'length': self.length}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.length = data.get('length', self.length)
