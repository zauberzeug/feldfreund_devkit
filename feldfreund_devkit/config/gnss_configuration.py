from dataclasses import dataclass

from rosys.geometry import Point3d, Pose


@dataclass(slots=True, kw_only=True)
class GnssConfiguration:
    """Configuration for the GNSS of the Field Friend robot.

    X, Y, Z are the position of the main GNSS antenna.
    The yaw is the direction to the auxiliary antenna.
    It should be 90°, but the offset is configured in the Septentrio software.

    Defaults:
        x: 0.041
        y: -0.255
        z: 0.6225
        yaw: 0.0
    """
    x: float = 0.041
    y: float = -0.255
    z: float = 0.6225
    yaw: float = 0.0

    @property
    def pose(self) -> Pose:
        return Pose(x=self.x, y=self.y, yaw=self.yaw)

    @property
    def point3d(self) -> Point3d:
        return Point3d(x=self.x, y=self.y, z=self.z)
