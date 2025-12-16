from dataclasses import dataclass
from typing import Self

from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline


@dataclass(slots=True, kw_only=True)
class DriveSegment(PathSegment):
    # TODO: move methods to rosys.driving.PathSegment
    use_implement: bool = False
    stop_at_end: bool = True

    @property
    def start(self) -> Pose:
        return self.spline.pose(t=0)

    @property
    def end(self) -> Pose:
        return self.spline.pose(t=1)

    @classmethod
    def from_poses(cls, start: Pose, end: Pose, *, use_implement: bool = False, backward: bool = False, stop_at_end: bool = True) -> Self:
        return cls(spline=Spline.from_poses(start, end, backward=backward), use_implement=use_implement, backward=backward, stop_at_end=stop_at_end)

    @classmethod
    def from_points(cls, start: Point, end: Point, *, use_implement: bool = False, backward: bool = False, stop_at_end: bool = True) -> Self:
        yaw = start.direction(end)
        start_pose = Pose(x=start.x, y=start.y, yaw=yaw)
        end_pose = Pose(x=end.x, y=end.y, yaw=yaw)
        return cls.from_poses(start_pose, end_pose, use_implement=use_implement, backward=backward, stop_at_end=stop_at_end)

    def __str__(self) -> str:
        return f'DriveSegment(start={self.start}, end={self.end}, backward={self.backward}, use_implement={self.use_implement}, stop_at_end={self.stop_at_end})'

    def __repr__(self) -> str:
        return self.__str__()
