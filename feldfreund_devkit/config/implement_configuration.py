

from dataclasses import dataclass, field

from rosys.geometry import Pose3d


@dataclass(kw_only=True)
class ImplementConfiguration:
    """Base configuration for all implements.

    Defaults:
        offset: Pose3d.zero
    """
    lizard_name: str
    display_name: str
    offset: Pose3d = field(default_factory=Pose3d.zero)
    work_radius: float
