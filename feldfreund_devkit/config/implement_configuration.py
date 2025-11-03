

from dataclasses import dataclass, field

from rosys.geometry import Point3d


@dataclass(kw_only=True)
class ImplementConfiguration:
    name: str
    offset: Point3d = field(default_factory=Point3d.zero)
