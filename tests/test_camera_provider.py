import pytest
import rosys
from rosys.geometry import Rectangle
from rosys.hardware import WheelsSimulation
from rosys.testing import forward
from rosys.vision import Calibration, ImageRotation, ImageSize, Intrinsics, SimulatedCalibratableCamera

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
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720)),
        front=MjpegCameraConfig(camera_id='mac-1', image_size=ImageSize(width=640, height=480), password='test-pw'),
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert provider.front is not None
    assert provider.back is None
    assert len(provider.cameras) == 2


async def test_multiple_mains(robot_locator):
    config = CameraConfiguration(
        main=[
            UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720)),
            UsbCameraConfig(camera_id='usb-1', image_size=ImageSize(width=1280, height=720)),
        ],
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert len(provider.mains) == 2
    assert provider.main is provider.mains[0]
    assert [cam.id for cam in provider.mains] == ['usb-0', 'usb-1']
    assert len(provider.main_slot_configs) == 2
    assert len(provider.cameras) == 2
    assert provider.slot_config('main') is provider.main_slot_configs[0]


async def test_empty_main_list(robot_locator):
    config = CameraConfiguration(main=[], front=None, back=None)
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is None
    assert provider.mains == []
    assert provider.main_slot_configs == []
    assert provider.slot_config('main') is None
    assert provider.cameras == {}


async def test_duplicate_camera_ids_across_slots_raise(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='cam-0', image_size=ImageSize(width=1280, height=720)),
        front=MjpegCameraConfig(camera_id='cam-0', image_size=ImageSize(width=640, height=480), password='test-pw'),
        back=None,
    )
    with pytest.raises(ValueError, match='Duplicate camera id'):
        CameraProvider(config, frame_provider=robot_locator)


async def test_duplicate_camera_ids_among_mains_raise(robot_locator):
    config = CameraConfiguration(
        main=[
            UsbCameraConfig(camera_id='cam-0', image_size=ImageSize(width=1280, height=720)),
            UsbCameraConfig(camera_id='cam-0', image_size=ImageSize(width=1280, height=720)),
        ],
        front=None,
        back=None,
    )
    with pytest.raises(ValueError, match='Duplicate camera id'):
        CameraProvider(config, frame_provider=robot_locator)


async def test_simulation_creates_simulated_cameras(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720)),
        front=RtspCameraConfig(camera_id='rtsp-1', mac='aa:bb:cc:dd:ee:ff', ip='192.168.1.1',
                               image_size=ImageSize(width=640, height=480)),
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
        main=UsbCameraConfig(camera_id='usb-0', calibration=calibration),
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
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720), crop=crop),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    main_config = provider.slot_config('main')
    assert main_config is not None
    assert main_config.crop == crop


async def test_rotation_config_accessible(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720),
                             rotation=ImageRotation.RIGHT),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    main_config = provider.slot_config('main')
    assert main_config is not None
    assert main_config.rotation == ImageRotation.RIGHT


async def test_cameras_connect_on_update(robot_locator):
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720)),
        front=None,
        back=None,
    )
    provider = CameraProvider(config, frame_provider=robot_locator)
    assert provider.main is not None
    assert not provider.main.is_connected
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert provider.main.is_connected


async def test_camera_without_auto_connect_stays_disconnected(robot_locator):
    """A camera configured with auto_connect=False is created but never connected by the repeat loop."""
    provider = _create_provider(robot_locator, auto_connect=False)
    assert 'usb-0' in provider.cameras
    assert provider.auto_connect == {'usb-0': False}
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert not provider.cameras['usb-0'].is_connected


async def test_enabling_auto_connect_connects_and_keeps_camera_connected(robot_locator):
    """Enabling auto-connect connects the camera immediately and the repeat loop keeps it connected."""
    provider = _create_provider(robot_locator, auto_connect=False)
    await provider.set_auto_connect('usb-0', True)
    assert provider.auto_connect['usb-0']
    assert provider.cameras['usb-0'].is_connected
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert provider.cameras['usb-0'].is_connected


async def test_disabling_auto_connect_disconnects_camera_for_good(robot_locator):
    """Disabling auto-connect disconnects the camera and the repeat loop does not reconnect it."""
    provider = _create_provider(robot_locator, auto_connect=True)
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert provider.cameras['usb-0'].is_connected
    await provider.set_auto_connect('usb-0', False)
    assert not provider.auto_connect['usb-0']
    assert not provider.cameras['usb-0'].is_connected
    await forward(provider.RECONNECT_INTERVAL + 1)
    assert not provider.cameras['usb-0'].is_connected


def _create_provider(robot_locator: RobotLocator, *, auto_connect: bool) -> CameraProvider:
    """Create a provider with a single main camera.

    :param robot_locator: Frame provider to link the camera extrinsics to.
    :param auto_connect: Desired connection state to configure for the camera.
    :return: The camera provider.
    """
    config = CameraConfiguration(
        main=UsbCameraConfig(camera_id='usb-0', image_size=ImageSize(width=1280, height=720),
                             auto_connect=auto_connect),
        front=None,
        back=None,
    )
    return CameraProvider(config, frame_provider=robot_locator)
