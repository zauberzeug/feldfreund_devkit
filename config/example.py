from rosys.vision import ImageSize

from feldfreund_devkit import secrets
from feldfreund_devkit.config import (
    BluetoothConfiguration,
    BumperConfiguration,
    CameraConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MjpegCameraConfig,
    RobotBrainConfiguration,
    TracksConfiguration,
    UsbCameraConfig,
)

config = FeldfreundConfiguration(
    robot_id='Example',
    bluetooth=BluetoothConfiguration(name='example', pin_code=123456),
    bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
    cameras=CameraConfiguration(
        main=UsbCameraConfig(camera_id='example-usb-0', image_size=ImageSize(width=1280, height=720), fps=10),
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
    wheels=TracksConfiguration(is_left_reversed=True,
                               is_right_reversed=False,
                               left_back_can_address=0x000,
                               left_front_can_address=0x100,
                               right_back_can_address=0x200,
                               right_front_can_address=0x300,
                               odrive_version=6),
)
