import logging

import rosys
from rosys.geometry import FrameProvider

from .config import (
    CameraConfiguration,
    CameraSlotConfig,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
)


class CameraProvider:
    """Config-driven camera provider with named role slots."""

    RECONNECT_INTERVAL = 10

    def __init__(self, config: CameraConfiguration | None, *, frame_provider: FrameProvider | None = None) -> None:
        self.log = logging.getLogger('feldfreund.camera_provider')
        self._config = config
        self.main = self._setup(config.main) if config and config.main else None
        self.front = self._setup(config.front) if config and config.front else None
        self.back = self._setup(config.back) if config and config.back else None
        self.left = self._setup(config.left) if config and config.left else None
        self.right = self._setup(config.right) if config and config.right else None

        if frame_provider is not None:
            self.set_frame_provider(frame_provider)

        if config is not None:
            rosys.on_repeat(self.update_device_list, self.RECONNECT_INTERVAL)
            rosys.on_shutdown(self.shutdown)

    @property
    def main_config(self) -> CameraSlotConfig | None:
        return self._config.main if self._config else None

    @property
    def front_config(self) -> CameraSlotConfig | None:
        return self._config.front if self._config else None

    @property
    def back_config(self) -> CameraSlotConfig | None:
        return self._config.back if self._config else None

    @property
    def left_config(self) -> CameraSlotConfig | None:
        return self._config.left if self._config else None

    @property
    def right_config(self) -> CameraSlotConfig | None:
        return self._config.right if self._config else None

    @property
    def cameras(self) -> dict[str, rosys.vision.CalibratableCamera]:
        """Required by rosys CalibratableCameraProvider protocol."""
        return {cam.id: cam for cam in (self.main, self.front, self.back, self.left, self.right) if cam is not None}

    @property
    def circle_sight_cameras(self) -> dict[str, rosys.vision.CalibratableCamera]:
        """Non-None front/back/left/right cameras keyed by direction name."""
        return {name: cam for name, cam in [
            ('front', self.front), ('back', self.back), ('left', self.left), ('right', self.right),
        ] if cam is not None}

    def set_frame_provider(self, frame_provider: FrameProvider) -> None:
        """Link all calibrated cameras to the given frame provider."""
        for camera in self.cameras.values():
            if camera.calibration is None:
                continue
            camera.calibration.extrinsics.in_frame(frame_provider.frame)

    def _setup(self, slot_config: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        camera = self._create_camera(slot_config)
        if slot_config.calibration is not None:
            camera.calibration = slot_config.calibration
        return camera

    def _create_camera(self, slot: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        if rosys.is_simulation():
            camera = rosys.vision.SimulatedCalibratableCamera(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
                color='#cccccc'
            )
        elif isinstance(slot, UsbCameraConfig):
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
                ip=slot.ip,
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
                ip=slot.ip,
                fps=slot.fps,
            )
        else:
            raise ValueError(f'Unknown camera slot type: {type(slot)}')

        return camera

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
