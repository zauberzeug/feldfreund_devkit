import logging

import rosys
from nicegui import ui
from rosys.geometry import FrameProvider

from .config import (
    CameraConfiguration,
    CameraSlotConfig,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
)
from .interface.components import status_bulb


class CalibratableUsbCamera(rosys.vision.CalibratableCamera, rosys.vision.UsbCamera):
    pass


class CalibratableRtspCamera(rosys.vision.CalibratableCamera, rosys.vision.RtspCamera):
    pass


class CalibratableMjpegCamera(rosys.vision.CalibratableCamera, rosys.vision.MjpegCamera):
    pass


class CameraProvider:

    def __init__(self, config: CameraConfiguration | None, *, frame_provider: FrameProvider | None = None, reconnect_interval: int = 10) -> None:
        """

        :param config: Optional camera configuration. If None, the provider will not attempt to connect to any cameras.
        :param frame_provider: Optional main frame to link calibrated cameras to.
        :param reconnect_interval: Interval in seconds to attempt reconnection to cameras.
        """
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
            rosys.on_repeat(self.update_device_list, reconnect_interval)
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
        slots = {'front': self.front, 'back': self.back, 'left': self.left, 'right': self.right}
        return {k: v for k, v in slots.items() if v is not None}

    def set_frame_provider(self, frame_provider: FrameProvider) -> None:
        """Link all calibrated cameras to the given frame provider."""
        for camera in self.cameras.values():
            if camera.calibration is None:
                continue
            camera.calibration.extrinsics.in_frame(frame_provider.frame)

    def _setup(self, slot_config: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        """Create a camera based on the given slot configuration, and apply calibration if available."""
        camera = self._create_camera(slot_config)
        if slot_config.calibration is not None:
            camera.calibration = slot_config.calibration
        return camera

    def _create_camera(self, slot: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        """Create a camera based on the given slot configuration."""
        if rosys.is_simulation():
            camera = rosys.vision.SimulatedCalibratableCamera(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
                color='#cccccc',
            )
        elif isinstance(slot, UsbCameraConfig):
            camera = CalibratableUsbCamera(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
                auto_exposure=slot.auto_exposure,
            )
        elif isinstance(slot, RtspCameraConfig):
            camera = CalibratableRtspCamera(
                id=slot.camera_id,
                mac=slot.mac,
                ip=slot.ip,
                fps=slot.fps,
                substream=slot.substream,
                avdec=slot.codec,
            )
        elif isinstance(slot, MjpegCameraConfig):
            camera = CalibratableMjpegCamera(
                id=slot.camera_id,
                username=slot.username,
                password=slot.password,
                ip=slot.ip,
                fps=slot.fps,
            )
        else:
            raise ValueError(f'Unknown camera slot type: {type(slot)}')
        self.log.debug('Created %s camera %s', self._camera_config_name(slot), camera.id)
        return camera

    def _camera_config_name(self, config: CameraSlotConfig) -> str:
        """Get a human-friendly camera type name based on the config class name, e.g. 'Usb' for UsbCameraConfig."""
        return type(config).__name__.removesuffix('CameraConfig').title()

    async def update_device_list(self) -> None:
        """Attempt to connect to all disconnected cameras."""
        for camera in self.cameras.values():
            if camera.is_connected:
                continue
            try:
                await camera.connect()
            except Exception:
                self.log.warning('Failed to connect camera %s', camera.id, exc_info=True)

    async def shutdown(self) -> None:
        """Disconnect all cameras on shutdown."""
        for camera in self.cameras.values():
            try:
                await camera.disconnect()
            except Exception:
                self.log.warning('Failed to disconnect camera %s', camera.id, exc_info=True)

    def developer_ui(self) -> None:
        slots = [('main', self.main), ('front', self.front), ('back', self.back),
                 ('left', self.left), ('right', self.right)]
        with ui.column():
            ui.label('Cameras').classes('text-center text-bold')
            with ui.grid(columns='auto auto auto auto').classes('gap-x-4 gap-y-1 items-center'):
                ui.label('Slot').classes('font-bold')
                ui.label('Connected').classes('font-bold')
                ui.label('Resolution').classes('font-bold')
                ui.label('Type').classes('font-bold')
                for name, camera in slots:
                    ui.label(name)
                    if camera is None:
                        status_bulb()
                        ui.label('—').classes('text-center')
                        ui.label('—').classes('text-center')
                    else:
                        slot_config: CameraSlotConfig = getattr(self, f'{name}_config')
                        status_bulb().bind_value_from(camera, 'is_connected')
                        resolution = ui.label('—')
                        ui.timer(5.0, lambda lbl=resolution, cam=camera: lbl.set_text(
                            f'{cam.latest_captured_image.size.width}x{cam.latest_captured_image.size.height}'
                            if cam.latest_captured_image else '—'))
                        ui.label(self._camera_config_name(slot_config))
