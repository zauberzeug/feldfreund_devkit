

from dataclasses import dataclass, field

from rosys.geometry import Pose3d


@dataclass(kw_only=True)
class ImplementConfiguration:
    """Base configuration for all implements.

    Defaults:
        default_offset: Pose3d.zero
    """
    lizard_name: str
    display_name: str
    default_offset: Pose3d = field(default_factory=Pose3d.zero)
    """Tool pose in the robot frame until an implement calibrates or persists its own."""
    work_radius: float
