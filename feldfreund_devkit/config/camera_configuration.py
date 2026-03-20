from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import rosys.vision


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
