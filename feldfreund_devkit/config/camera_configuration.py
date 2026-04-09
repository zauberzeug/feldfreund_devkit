from dataclasses import dataclass
from typing import Literal

from rosys.geometry import Pose3d, Rectangle, Rotation
from rosys.vision import Calibration, ImageRotation, ImageSize, Intrinsics


def create_calibration(*, fx: float,
                       fy: float,
                       cx: float,
                       cy: float,
                       distortion: list[float],
                       width: int,
                       height: int,
                       x: float,
                       y: float,
                       z: float,
                       roll: float,
                       pitch: float,
                       yaw: float) -> Calibration:
    """Helper function to create a camera calibration from intrinsic and extrinsic parameters."""
    intrinsics = Intrinsics(matrix=_create_camera_matrix(fx=fx, fy=fy, cx=cx, cy=cy),
                            distortion=distortion,
                            size=ImageSize(width=width, height=height))
    extrinsics = Pose3d(x=x, y=y, z=z, rotation=Rotation.from_euler(roll=roll, pitch=pitch, yaw=yaw))
    return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)


def _create_camera_matrix(*, fx: float, fy: float, cx: float, cy: float) -> list[list[float]]:
    """Helper function to create a camera matrix from focal lengths and principal point."""
    return [
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ]


@dataclass(kw_only=True)
class CameraSlotConfig:
    """Base configuration shared by all camera types.

    ``width`` and ``height`` are derived properties: they come from
    ``calibration.intrinsics.size`` when a calibration is provided,
    otherwise from ``image_size``.  At least one of the two must be set.

    Defaults:
        fps: 10
        rotation: ImageRotation.NONE
        crop: None
        calibration: None
        image_size: None
    """
    camera_id: str
    fps: int = 10
    rotation: ImageRotation = ImageRotation.NONE
    crop: Rectangle | None = None
    calibration: Calibration | None = None
    image_size: ImageSize | None = None

    def __post_init__(self) -> None:
        if self.calibration is None and self.image_size is None:
            raise ValueError('either calibration or image_size must be provided')

    @property
    def camera_kwargs(self) -> dict:
        return {'id': self.camera_id, 'fps': self.fps}

    @property
    def width(self) -> int:
        if self.calibration is not None:
            return self.calibration.intrinsics.size.width
        assert self.image_size is not None
        return self.image_size.width

    @property
    def height(self) -> int:
        if self.calibration is not None:
            return self.calibration.intrinsics.size.height
        assert self.image_size is not None
        return self.image_size.height


@dataclass(kw_only=True)
class UsbCameraConfig(CameraSlotConfig):
    """Configuration for a USB camera.

    Defaults:
        auto_exposure: True
    """
    auto_exposure: bool = True

    @property
    def camera_kwargs(self) -> dict:
        return {**super().camera_kwargs, 'width': self.width, 'height': self.height,
                'auto_exposure': self.auto_exposure}


@dataclass(kw_only=True)
class RtspCameraConfig(CameraSlotConfig):
    """Configuration for an RTSP camera.

    Defaults:
        codec: 'h265'
        substream: 0
    """
    mac: str
    ip: str
    codec: Literal['h264', 'h265'] = 'h265'
    substream: int = 0

    @property
    def camera_kwargs(self) -> dict:
        return {**super().camera_kwargs, 'mac': self.mac, 'ip': self.ip,
                'substream': self.substream, 'avdec': self.codec}


@dataclass(kw_only=True)
class MjpegCameraConfig(CameraSlotConfig):
    """Configuration for an MJPEG camera.

    Defaults:
        username: 'root'
        password: 'zauberzg!'
    """
    ip: str = '192.168.42.3'
    username: str = 'root'
    password: str = 'zauberzg!'

    @property
    def camera_kwargs(self) -> dict:
        return {**super().camera_kwargs, 'username': self.username, 'password': self.password,
                'ip': self.ip}


@dataclass(kw_only=True)
class CameraConfiguration:
    """Container of named camera slots for a Feldfreund robot.

    Defaults:
        left: None
        right: None
    """
    main: CameraSlotConfig | None
    front: CameraSlotConfig | None
    back: CameraSlotConfig | None
    left: CameraSlotConfig | None = None
    right: CameraSlotConfig | None = None
