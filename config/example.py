from rosys.geometry import Rectangle
from rosys.vision import ImageSize

from feldfreund_devkit.config import (
    BluetoothConfiguration,
    BumperConfiguration,
    CameraConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MjpegCameraConfig,
    ODriveTracksConfiguration,
    RobotBrainConfiguration,
    Secrets,
    UsbCameraConfig,
    create_calibration,
)


def build_config(secrets: Secrets) -> FeldfreundConfiguration:
    return FeldfreundConfiguration(
        robot_id='Example',
        bluetooth=BluetoothConfiguration(name='example', pin_code=123456),
        bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
        cameras=CameraConfiguration(
            # streams the full sensor and crops to the working area; the calibration was fit at 1280x960
            main=UsbCameraConfig(camera_id='example-usb-0', fps=10,
                                 calibration=create_calibration(fx=720.0, fy=720.0, cx=640.0, cy=480.0,
                                                                distortion=[0.0, 0.0, 0.0, 0.0, 0.0],
                                                                width=1280, height=960,
                                                                x=0.4, y=0.0, z=0.6,
                                                                roll=3.14, pitch=0.0, yaw=-1.57),
                                 stream_size=ImageSize(width=2560, height=1920),
                                 crop=Rectangle(x=640, y=480, width=1280, height=960)),
            front=MjpegCameraConfig(camera_id='example-mac-4', image_size=ImageSize(width=1280, height=720),
                                    password=secrets.MJPEG_CAMERA_PASSWORD),
            back=MjpegCameraConfig(camera_id='example-mac-3', image_size=ImageSize(width=1280, height=720),
                                   password=secrets.MJPEG_CAMERA_PASSWORD),
            right=MjpegCameraConfig(camera_id='example-mac-1', image_size=ImageSize(width=1280, height=720),
                                    password=secrets.MJPEG_CAMERA_PASSWORD),
            left=MjpegCameraConfig(camera_id='example-mac-2', image_size=ImageSize(width=1280, height=720),
                                   password=secrets.MJPEG_CAMERA_PASSWORD),
        ),
        flashlight=FlashlightConfiguration(),
        gnss=GnssConfiguration(),
        implement=None,
        imu=ImuConfiguration(),
        robot_brain=RobotBrainConfiguration(name='rbexample', nand=True),
        wheels=ODriveTracksConfiguration(is_left_reversed=True,
                                         is_right_reversed=False,
                                         left_back_can_address=0x000,
                                         left_front_can_address=0x100,
                                         right_back_can_address=0x200,
                                         right_front_can_address=0x300,
                                         odrive_version=6),
    )
