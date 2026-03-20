import pytest
import rosys
from rosys.hardware import WheelsSimulation
from rosys.testing import forward
from rosys.vision import Calibration, Intrinsics, SimulatedCalibratableCamera

from feldfreund_devkit.camera_provider import CameraProvider
from feldfreund_devkit.config import (
    CameraConfiguration,
    CropConfiguration,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
)
from feldfreund_devkit.robot_locator import RobotLocator


@pytest.fixture
async def robot_locator(rosys_integration) -> RobotLocator:
    rosys.enter_simulation()
    return RobotLocator(WheelsSimulation())


async def test_none_config(robot_locator):
    provider = CameraProvider(None, robot_locator=robot_locator)
    assert provider.main is None
    assert provider.front is None
    assert provider.cameras == {}


async def test_slots_assigned(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
        front=MjpegCameraConfig(camera_id='mac-1', width=640, height=480),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert provider.main is not None
    assert provider.front is not None
    assert provider.back is None
    assert len(provider.cameras) == 2


async def test_simulation_creates_simulated_cameras(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
        front=RtspCameraConfig(camera_id='rtsp-1', width=640, height=480),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert isinstance(provider.main, SimulatedCalibratableCamera)
    assert isinstance(provider.front, SimulatedCalibratableCamera)


async def test_calibration_applied(robot_locator):
    calibration = Calibration(
        intrinsics=Intrinsics.create_default(width=1280, height=720, focal_length=800),
    )
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720, calibration=calibration),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert provider.main is not None
    assert provider.main.calibration is not None
    assert provider.main.calibration.extrinsics.frame_id == robot_locator.pose_frame.id


async def test_crop_applied(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(
            camera_id='usb-0', width=1280, height=720,
            crop=CropConfiguration(left=100, right=100, up=50, down=50),
        ),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert provider.main is not None
    assert provider.main.crop is not None
    assert provider.main.crop.x == 100
    assert provider.main.crop.y == 50
    assert provider.main.crop.width == 1080
    assert provider.main.crop.height == 620


async def test_rotation_applied(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720, rotation=90),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert provider.main is not None
    assert provider.main.rotation_angle == 90


async def test_cameras_connect_on_update(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
    )
    provider = CameraProvider(config, robot_locator=robot_locator)
    assert provider.main is not None
    assert not provider.main.is_connected
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert provider.main.is_connected
