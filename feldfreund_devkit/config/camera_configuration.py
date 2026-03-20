from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import rosys.vision
from rosys.geometry import Pose3d, Rotation


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
                       yaw: float) -> rosys.vision.Calibration:
    """Helper function to create a camera calibration from intrinsic and extrinsic parameters."""
    intrinsics = rosys.vision.Intrinsics(matrix=_create_camera_matrix(fx=fx, fy=fy, cx=cx, cy=cy),
                                         distortion=distortion,
                                         size=rosys.vision.ImageSize(width=width, height=height))
    extrinsics = Pose3d(x=x, y=y, z=z, rotation=Rotation.from_euler(roll=roll, pitch=pitch, yaw=yaw))
    return rosys.vision.Calibration(intrinsics=intrinsics, extrinsics=extrinsics)


def _create_camera_matrix(*, fx: float, fy: float, cx: float, cy: float) -> list[list[float]]:
    """Helper function to create a camera matrix from focal lengths and principal point."""
    return [
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ]


@dataclass(slots=True, kw_only=True)
class CropConfiguration:
    """Configuration for the cropping of a camera image.

    All values are in pixels.
    """
    left: int
    right: int
    up: int
    down: int


@dataclass(slots=True, kw_only=True)
class CameraSlotConfig:
    """Base configuration shared by all camera types.

    Defaults:
        fps: 10
        rotation: 0
        crop: None
        calibration: None
    """
    camera_id: str
    width: int
    height: int
    fps: int = 10
    rotation: int = 0
    crop: CropConfiguration | None = None
    calibration: rosys.vision.Calibration | None = None


@dataclass(slots=True, kw_only=True)
class UsbCameraConfig(CameraSlotConfig):
    """Configuration for a USB camera.

    Defaults:
        auto_exposure: True
    """
    auto_exposure: bool = True


@dataclass(slots=True, kw_only=True)
class RtspCameraConfig(CameraSlotConfig):
    """Configuration for an RTSP camera.

    Defaults:
        codec: 'h265'
        substream: 0
    """
    ip: str
    codec: Literal['h264', 'h265'] = 'h265'
    substream: int = 0


@dataclass(slots=True, kw_only=True)
class MjpegCameraConfig(CameraSlotConfig):
    """Configuration for an MJPEG camera.

    Defaults:
        username: 'root'
        password: 'zauberzg!'
    """
    username: str = 'root'
    password: str = 'zauberzg!'


@dataclass(slots=True, kw_only=True)
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
