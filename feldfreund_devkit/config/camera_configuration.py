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

    ``width`` and ``height`` are derived properties describing the images the camera delivers:
    the ``crop`` size when a crop is configured, otherwise the stream size.
    At least one of ``calibration`` and ``image_size`` must be set.

    ``stream_size`` lets the camera stream at a different resolution than the ``calibration``
    was fit at (same field of view); the calibration is scaled to match, so it requires one.
    ``crop`` is the region cut out of the stream, in stream pixel coordinates. It is applied to
    the captured images via ``TransformableCamera`` and shifts the calibration's principal point.
    ``rotation`` is currently not passed to the cameras.

    ``auto_connect`` set to ``False`` keeps the camera disconnected until the connection is
    requested explicitly, for example with the switch in the camera developer UI.

    Defaults:
        fps: 10
        rotation: ImageRotation.NONE
        stream_size: None (the calibration size or ``image_size``)
        crop: None
        calibration: None
        image_size: None
        auto_connect: True
    """
    camera_id: str
    fps: int = 10
    rotation: ImageRotation = ImageRotation.NONE
    stream_size: ImageSize | None = None
    crop: Rectangle | None = None
    calibration: Calibration | None = None
    image_size: ImageSize | None = None
    auto_connect: bool = True

    def __post_init__(self) -> None:
        if self.calibration is None and self.image_size is None:
            raise ValueError('either calibration or image_size must be provided')
        if self.stream_size is not None and self.calibration is None:
            raise ValueError('stream_size requires a calibration to derive the stream calibration from')
        if self.calibration is not None:
            intrinsics = self.calibration.intrinsics
            if self.stream_size is None:
                self.stream_size = intrinsics.size
            else:
                intrinsics = intrinsics.scale(self.stream_size)
            if self.crop is not None:
                intrinsics = intrinsics.crop(self.crop)
            if intrinsics is not self.calibration.intrinsics:
                self.calibration = Calibration(intrinsics=intrinsics, extrinsics=self.calibration.extrinsics)
        else:
            self.stream_size = self.stream_size or self.image_size

    @property
    def camera_kwargs(self) -> dict:
        return {'id': self.camera_id, 'fps': self.fps, 'connect_after_init': self.auto_connect,
                'crop': self.crop}

    @property
    def width(self) -> int:
        if self.crop is not None:
            return int(self.crop.width)
        assert self.stream_size is not None
        return self.stream_size.width

    @property
    def height(self) -> int:
        if self.crop is not None:
            return int(self.crop.height)
        assert self.stream_size is not None
        return self.stream_size.height


@dataclass(kw_only=True)
class UsbCameraConfig(CameraSlotConfig):
    """Configuration for a USB camera.

    Defaults:
        auto_exposure: True
    """
    auto_exposure: bool = True

    @property
    def camera_kwargs(self) -> dict:
        assert self.stream_size is not None
        return {**super().camera_kwargs, 'resolution': (self.stream_size.width, self.stream_size.height),
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

    def __post_init__(self) -> None:
        if self.stream_size is not None:
            raise ValueError('RTSP cameras cannot set a stream resolution; the encoder substream determines it')
        super().__post_init__()

    @property
    def camera_kwargs(self) -> dict:
        return {**super().camera_kwargs, 'mac': self.mac, 'ip': self.ip,
                'substream': self.substream, 'avdec': self.codec}


@dataclass(kw_only=True)
class MjpegCameraConfig(CameraSlotConfig):
    """Configuration for an MJPEG camera.

    The password can be added to the .env file as MJPEG_CAMERA_PASSWORD and then used with secrets.py.

    Defaults:
        ip: '192.168.42.3'
        username: 'root'
    """
    ip: str = '192.168.42.3'
    username: str = 'root'
    password: str

    @property
    def camera_kwargs(self) -> dict:
        assert self.stream_size is not None
        return {**super().camera_kwargs, 'username': self.username, 'password': self.password,
                'ip': self.ip, 'resolution': (self.stream_size.width, self.stream_size.height)}


@dataclass(kw_only=True)
class CameraConfiguration:
    """Container of named camera slots for a Feldfreund robot.

    ``main`` may be a single ``CameraSlotConfig`` or a list of them. The list form configures
    multiple main cameras; ``CameraProvider.main`` then refers to the first, while
    ``CameraProvider.mains`` exposes all of them.

    Defaults:
        left: None
        right: None
    """
    main: CameraSlotConfig | list[CameraSlotConfig] | None
    front: CameraSlotConfig | None
    back: CameraSlotConfig | None
    left: CameraSlotConfig | None = None
    right: CameraSlotConfig | None = None
