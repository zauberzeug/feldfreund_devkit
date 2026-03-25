from dataclasses import dataclass

from rosys.geometry import Point, Pose


@dataclass(slots=True, kw_only=True)
class RobotFootprint:
    """Physical extent of the robot as distances from the center of motion.

    The center of motion is not necessarily the geometric center of the robot frame.
    Each value represents the distance from the center of motion to that edge.

    Defaults:
        front: 0.5295
        rear: 0.343
        left: 0.33274
        right: 0.33274
    """
    front: float = 0.5295
    rear: float = 0.343
    left: float = 0.33274
    right: float = 0.33274

    def corners_at_pose(self, pose: Pose) -> list[Point]:
        """Return the 4 footprint corners at the given pose, rotated by the pose's yaw.

        Order: front-left, front-right, rear-right, rear-left.
        """
        return [
            pose.transform(Point(x=self.front, y=self.left)),
            pose.transform(Point(x=self.front, y=-self.right)),
            pose.transform(Point(x=-self.rear, y=-self.right)),
            pose.transform(Point(x=-self.rear, y=self.left)),
        ]
