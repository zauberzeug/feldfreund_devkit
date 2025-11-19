from dataclasses import dataclass
from typing import Literal

from rosys.geometry import Rectangle


@dataclass(kw_only=True)
class CropConfiguration:
    """Configuration for the cropping of the camera of the Field Friend robot."""
    left: int
    right: int
    up: int
    down: int


@dataclass(kw_only=True)
class CircleSightPositions:
    """Configuration for the positions of the 4 cameras.

    Defaults:
        right = '-1'
        back = '-2'
        front = '-3'
        left = '-4'
    """
    right: str = '-1'
    back: str = '-2'
    front: str = '-3'
    left: str = '-4'


@dataclass(kw_only=True)
class CameraConfiguration:
    """Configuration for the camera of the Field Friend robot.

    Attributes:
        camera_type: default = 'CalibratableUsbCamera'
        auto_exposure: default = True
        rotation: default = 0
        fps: default = 10
        crop: default = None
    """
    width: int
    height: int
    camera_type: Literal['CalibratableUsbCamera'] = 'CalibratableUsbCamera'
    auto_exposure: bool = True
    rotation: int = 0
    fps: int = 10
    crop: CropConfiguration | None = None

    @property
    def crop_rectangle(self) -> Rectangle | None:
        """get a rectangle based on the crop values (left, right, up, down) of the config"""
        if self.crop is None:
            return None
        new_width = self.width - (self.crop.left + self.crop.right)
        new_height = self.height - (self.crop.up + self.crop.down)
        return Rectangle(x=self.crop.left, y=self.crop.up, width=new_width, height=new_height)

    @property
    def parameters(self) -> dict:
        return {
            'width': self.width,
            'height': self.height,
            'auto_exposure': self.auto_exposure,
            'fps': self.fps,
        }
