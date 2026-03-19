import logging

import rosys
from rosys.geometry import Rectangle

from .config import (
    CameraConfiguration,
    CameraSlotConfig,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
)
from .robot_locator import RobotLocator


class CameraProvider:
    """Config-driven camera provider with named role slots."""

    RECONNECT_INTERVAL = 10

    def __init__(self, config: CameraConfiguration | None, *, robot_locator: RobotLocator) -> None:
        self.log = logging.getLogger('feldfreund.camera_provider')
        self.main: rosys.vision.CalibratableCamera | None = None
        self.front: rosys.vision.CalibratableCamera | None = None
        self.back: rosys.vision.CalibratableCamera | None = None
        self.left: rosys.vision.CalibratableCamera | None = None
        self.right: rosys.vision.CalibratableCamera | None = None

        if config is None:
            return

        for role in ('main', 'front', 'back', 'left', 'right'):
            slot: CameraSlotConfig | None = getattr(config, role, None)
            if slot is None:
                continue
            camera = self._create_camera(slot)
            setattr(self, role, camera)

            if slot.calibration is not None:
                camera.calibration = slot.calibration
                camera.calibration.extrinsics.in_frame(robot_locator.pose_frame)

        rosys.on_repeat(self.update_device_list, self.RECONNECT_INTERVAL)
        rosys.on_shutdown(self.shutdown)

    def _create_camera(self, slot: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        if rosys.is_simulation():
            return rosys.vision.SimulatedCalibratableCamera(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
            )

        if isinstance(slot, UsbCameraConfig):
            camera_class = type('CalibratableCamera', (rosys.vision.CalibratableCamera, rosys.vision.UsbCamera), {})
            camera = camera_class(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
                auto_exposure=slot.auto_exposure,
            )
        elif isinstance(slot, RtspCameraConfig):
            camera_class = type('CalibratableCamera', (rosys.vision.CalibratableCamera, rosys.vision.RtspCamera), {})
            camera = camera_class(
                id=slot.camera_id,
                fps=slot.fps,
                substream=slot.substream,
                avdec=slot.codec,
            )
        elif isinstance(slot, MjpegCameraConfig):
            camera_class = type('CalibratableCamera', (rosys.vision.CalibratableCamera, rosys.vision.MjpegCamera), {})
            camera = camera_class(
                id=slot.camera_id,
                username=slot.username,
                password=slot.password,
                fps=slot.fps,
            )
        else:
            raise ValueError(f'Unknown camera slot type: {type(slot)}')

        if slot.crop is not None:
            new_width = slot.width - (slot.crop.left + slot.crop.right)
            new_height = slot.height - (slot.crop.up + slot.crop.down)
            camera.crop = Rectangle(x=slot.crop.left, y=slot.crop.up, width=new_width, height=new_height)
        if slot.rotation != 0:
            camera.rotation_angle = slot.rotation

        return camera

    @property
    def cameras(self) -> dict[str, rosys.vision.CalibratableCamera]:
        return {role: cam for role in ('main', 'front', 'back', 'left', 'right')
                if (cam := getattr(self, role)) is not None}

    @property
    def first_connected_camera(self) -> rosys.vision.CalibratableCamera | None:
        return next((camera for camera in self.cameras.values() if camera.is_connected), None)

    async def update_device_list(self) -> None:
        for camera in self.cameras.values():
            if not camera.is_connected:
                try:
                    await camera.connect()
                except Exception:
                    self.log.warning('Failed to connect camera %s', camera.id, exc_info=True)

    async def shutdown(self) -> None:
        for camera in self.cameras.values():
            try:
                await camera.disconnect()
            except Exception:
                self.log.warning('Failed to disconnect camera %s', camera.id, exc_info=True)
