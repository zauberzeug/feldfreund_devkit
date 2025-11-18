import numpy as np
import pytest
from rosys.geometry import Pose
from rosys.hardware import ImuSimulation
from rosys.testing import forward


@pytest.mark.parametrize('roll_direction', (-1, 1))
@pytest.mark.parametrize('pitch_direction', (-1, 1))
async def test_height_correction(devkit_system, imu: ImuSimulation, roll_direction: int, pitch_direction: int):
    # pylint: disable=protected-access
    imu.roll = np.deg2rad(10.0) * roll_direction
    imu.pitch = np.deg2rad(10.0) * pitch_direction
    await forward(1)
    corrected_pose = devkit_system.robot_locator._correct_gnss_with_imu(Pose(x=0.0, y=0.0, yaw=0.0))
    assert corrected_pose.x == pytest.approx(pitch_direction * -0.108, abs=0.01)
    assert corrected_pose.y == pytest.approx(roll_direction * 0.112, abs=0.01)
