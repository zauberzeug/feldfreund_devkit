from __future__ import annotations

from typing import TYPE_CHECKING

from rosys.geometry import Pose
from rosys.hardware import WheelsSimulation

if TYPE_CHECKING:
    from ..system import System


def set_robot_pose(system: System, pose: Pose):
    # pylint: disable=protected-access
    assert isinstance(system.feldfreund.wheels, WheelsSimulation)
    system.robot_locator._x[0, 0] = pose.x
    system.robot_locator._x[1, 0] = pose.y
    system.robot_locator._x[2, 0] = pose.yaw
    system.feldfreund.wheels.pose.x = pose.x
    system.feldfreund.wheels.pose.y = pose.y
    system.feldfreund.wheels.pose.yaw = pose.yaw
