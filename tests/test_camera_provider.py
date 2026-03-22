import pytest
import rosys
from rosys.geometry import Rectangle
from rosys.hardware import WheelsSimulation
from rosys.testing import forward
from rosys.vision import Calibration, ImageRotation, Intrinsics, SimulatedCalibratableCamera

from feldfreund_devkit.camera_provider import CameraProvider
from feldfreund_devkit.config import (
    CameraConfiguration,
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
    provider = CameraProvider(None, frame_provider=robot_locator)
    assert provider.main is None
    assert provider.front is None
    assert provider.cameras == {}


async def test_slots_assigned(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
        front=MjpegCameraConfig(camera_id='mac-1', width=640, height=480),
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert provider.front is not None
    assert provider.back is None
    assert len(provider.cameras) == 2


async def test_simulation_creates_simulated_cameras(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
        front=RtspCameraConfig(camera_id='rtsp-1', ip='192.168.1.1', width=640, height=480),
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert isinstance(provider.main, SimulatedCalibratableCamera)
    assert isinstance(provider.front, SimulatedCalibratableCamera)


async def test_calibration_applied(robot_locator):
    calibration = Calibration(
        intrinsics=Intrinsics.create_default(width=1280, height=720, focal_length=800),
    )
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720, calibration=calibration),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert provider.main.calibration is not None
    assert provider.main.calibration.extrinsics.frame_id == robot_locator.frame.id


async def test_crop_config_accessible(robot_locator):
    crop = Rectangle(x=100, y=50, width=1080, height=620)
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720, crop=crop),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert provider.main_config is not None
    assert provider.main_config.crop == crop


async def test_rotation_config_accessible(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720, rotation=ImageRotation.RIGHT),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert provider.main_config is not None
    assert provider.main_config.rotation == ImageRotation.RIGHT


async def test_cameras_connect_on_update(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', width=1280, height=720),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert not provider.main.is_connected
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert provider.main.is_connected
