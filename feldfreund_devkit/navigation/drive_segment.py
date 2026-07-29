from dataclasses import dataclass
from typing import Self

from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline


@dataclass(slots=True, kw_only=True)
class DriveSegment(PathSegment):
    """A path segment with implement usage, stop behavior and speed configuration.

    :param speed_limit: forward speed cap while on this segment, or ``None`` for no constraint of
        its own. Set it where the segment itself demands a speed -- docking, turns. It caps rather
        than overrides: the robot never drives faster than the navigation's own limit.
    """
    # TODO: move methods to rosys.driving.PathSegment
    use_implement: bool = False
    stop_at_end: bool = True
    speed_limit: float | None = None

    @property
    def start(self) -> Pose:
        return self.spline.pose(t=0)

    @property
    def end(self) -> Pose:
        return self.spline.pose(t=1)

    @classmethod
    def from_poses(cls, start: Pose, end: Pose, *, use_implement: bool = False, backward: bool = False,
                   stop_at_end: bool = True, speed_limit: float | None = None) -> Self:
        return cls(spline=Spline.from_poses(start, end, backward=backward), use_implement=use_implement,
                   backward=backward, stop_at_end=stop_at_end, speed_limit=speed_limit)

    @classmethod
    def from_points(cls, start: Point, end: Point, *, use_implement: bool = False, backward: bool = False,
                    stop_at_end: bool = True, speed_limit: float | None = None) -> Self:
        yaw = start.direction(end)
        start_pose = Pose(x=start.x, y=start.y, yaw=yaw)
        end_pose = Pose(x=end.x, y=end.y, yaw=yaw)
        return cls.from_poses(start_pose, end_pose, use_implement=use_implement, backward=backward,
                              stop_at_end=stop_at_end, speed_limit=speed_limit)

    def __str__(self) -> str:
        return (f'DriveSegment(start={self.start}, end={self.end}, backward={self.backward}, '
                f'use_implement={self.use_implement}, stop_at_end={self.stop_at_end}, speed_limit={self.speed_limit})')

    def __repr__(self) -> str:
        return self.__str__()
