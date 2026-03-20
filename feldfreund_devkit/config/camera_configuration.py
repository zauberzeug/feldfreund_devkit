from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import rosys.vision


@dataclass(kw_only=True)
class CropConfiguration:
    """Configuration for the cropping of the camera of the Feldfreund robot."""
    left: int
    right: int
    up: int
    down: int


@dataclass(kw_only=True)
class CameraSlotConfig:
    """Base configuration shared by all camera types."""
    camera_id: str
    width: int
    height: int
    fps: int = 10
    rotation: int = 0
    crop: CropConfiguration | None = None
    calibration: rosys.vision.Calibration | None = None


@dataclass(kw_only=True)
class UsbCameraConfig(CameraSlotConfig):
    """Configuration for a USB camera."""
    auto_exposure: bool = True


@dataclass(kw_only=True)
class RtspCameraConfig(CameraSlotConfig):
    """Configuration for an RTSP camera."""
    ip: str
    codec: Literal['h264', 'h265'] = 'h265'
    substream: int = 0


@dataclass(kw_only=True)
class MjpegCameraConfig(CameraSlotConfig):
    """Configuration for an MJPEG camera."""
    username: str = 'root'
    password: str = 'zauberzg!'


@dataclass(kw_only=True)
class CameraConfiguration:
    """Container of named camera slots for a Feldfreund robot."""
    main: CameraSlotConfig | None
    front: CameraSlotConfig | None
    back: CameraSlotConfig | None
    left: CameraSlotConfig | None = None
    right: CameraSlotConfig | None = None
