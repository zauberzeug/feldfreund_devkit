from __future__ import annotations

import logging
from typing import Literal

import rosys
from nicegui import ui
from rosys.geometry import FrameProvider, Pose3d, Rotation
from rosys.vision import CalibratableCamera
from rosys.vision.mjpeg_camera.vendors import VendorType as MjpegVendorType
from rosys.vision.mjpeg_camera.vendors import mac_to_vendor as mjpeg_mac_to_vendor
from rosys.vision.rtsp_camera.arp_scan import find_cameras
from rosys.vision.rtsp_camera.vendors import mac_to_vendor as rtsp_mac_to_vendor
from rosys.vision.usb_camera.usb_camera_scanner import scan_for_connected_devices

from .config import (
    CameraConfiguration,
    CameraSlotConfig,
    MjpegCameraConfig,
    RtspCameraConfig,
    UsbCameraConfig,
)
from .interface.components import status_bulb


class PoseMetadataMixin(rosys.vision.CalibratableCamera):
    """Attaches the resolved camera pose to each captured image's metadata."""

    def _add_image(self, image: rosys.vision.Image) -> None:
        if self.calibration is not None:
            pose = self.calibration.extrinsics.resolve()
            image.metadata['pose'] = Pose3d(x=pose.x, y=pose.y, z=pose.z,
                                            rotation=Rotation(R=[row[:] for row in pose.rotation.R]))
        super()._add_image(image)


class CalibratableUsbCamera(PoseMetadataMixin, rosys.vision.UsbCamera):
    pass


class CalibratableRtspCamera(PoseMetadataMixin, rosys.vision.RtspCamera):
    pass


class CalibratableMjpegCamera(PoseMetadataMixin, rosys.vision.MjpegCamera):
    pass


class SimulatedCalibratableCamera(PoseMetadataMixin, rosys.vision.SimulatedCalibratableCamera):  # pylint: disable=too-many-ancestors
    pass


CameraPosition = Literal['main', 'front', 'back', 'left', 'right']


class CameraProvider:

    RECONNECT_INTERVAL = 10

    def __init__(self, config: CameraConfiguration | None, *, frame_provider: FrameProvider | None = None) -> None:
        """Config-driven camera provider with named role slots (main, front, back, left, right).

        :param config: Optional camera configuration. If None, the provider will not attempt to connect to any cameras.
        :param frame_provider: Optional frame provider to link calibrated camera extrinsics to.
        """
        self.log = logging.getLogger('feldfreund.camera_provider')
        self._config = config
        self._should_be_connected: dict[str, bool] = {}
        self.mains: list[rosys.vision.CalibratableCamera] = self._setup_mains()
        self.front = self._setup('front')
        self.back = self._setup('back')
        self.left = self._setup('left')
        self.right = self._setup('right')
        cameras = [cam for cam in (*self.mains, self.front, self.back, self.left, self.right) if cam is not None]
        ids = [cam.id for cam in cameras]
        duplicate_ids = sorted({cam_id for cam_id in ids if ids.count(cam_id) > 1})
        if duplicate_ids:
            raise ValueError(f'Duplicate camera id(s) in configuration: {", ".join(duplicate_ids)}')
        self._cameras = {cam.id: cam for cam in cameras}

        if frame_provider is not None:
            self.set_frame_provider(frame_provider)

        if config is not None:
            rosys.on_repeat(self.update_device_list, self.RECONNECT_INTERVAL)
            rosys.on_shutdown(self.shutdown)

    def slot_config(self, name: CameraPosition) -> CameraSlotConfig | None:
        """Get the CameraSlotConfig for a given position.

        For ``'main'``, returns the first configured main; use ``main_slot_configs`` for the full list.
        """
        if self._config is None:
            return None
        match name:
            case 'main':
                configs = self.main_slot_configs
                return configs[0] if configs else None
            case 'front':
                return self._config.front
            case 'back':
                return self._config.back
            case 'left':
                return self._config.left
            case 'right':
                return self._config.right
        return None

    @property
    def main_slot_configs(self) -> list[CameraSlotConfig]:
        """Normalized list of main camera slot configurations (empty when none configured)."""
        if self._config is None or self._config.main is None:
            return []
        return list(self._config.main) if isinstance(self._config.main, list) else [self._config.main]

    @property
    def main(self) -> rosys.vision.CalibratableCamera | None:
        """Primary main camera — the first of ``self.mains`` (used where a single camera is needed)."""
        return self.mains[0] if self.mains else None

    @property
    def cameras(self) -> dict[str, rosys.vision.CalibratableCamera]:
        """Required by rosys CalibratableCameraProvider protocol."""
        return self._cameras

    @property
    def circle_sight_cameras(self) -> dict[str, rosys.vision.CalibratableCamera]:
        """Non-None front/back/left/right cameras keyed by direction name."""
        slots = {'front': self.front, 'back': self.back, 'left': self.left, 'right': self.right}
        return {k: v for k, v in slots.items() if v is not None}

    @property
    def should_be_connected(self) -> dict[str, bool]:
        """Whether each camera is supposed to be connected, initialized from the slot configurations.

        The returned dictionary is a copy; use ``set_connected`` to change a state so that the
        camera is connected or disconnected accordingly. The state is runtime-only: it is not
        persisted and falls back to the configuration on restart.
        """
        return dict(self._should_be_connected)

    def set_frame_provider(self, frame_provider: FrameProvider) -> None:
        """Link all calibrated cameras to the given frame provider."""
        for camera in self.cameras.values():
            if camera.calibration is None:
                continue
            camera.calibration.extrinsics.in_frame(frame_provider.frame)

    async def set_connected(self, camera_id: str, connected: bool) -> None:
        """Connect or disconnect a camera and remember that this is how it should stay.

        :param camera_id: Id of the camera to connect or disconnect.
        :param connected: Whether the camera should be connected.
        :raises ValueError: If no camera with the given id is configured.
        """
        if camera_id not in self._cameras:
            raise ValueError(f'Unknown camera id: {camera_id} (available: {", ".join(sorted(self._cameras))})')
        self._should_be_connected[camera_id] = connected
        camera = self._cameras[camera_id]
        if connected:
            await camera.connect()
        else:
            await camera.disconnect()

    def _setup(self, name: CameraPosition) -> rosys.vision.CalibratableCamera | None:
        """Create a camera for the given position, and apply calibration if available."""
        slot_config = self.slot_config(name)
        if not slot_config:
            return None
        return self._build_camera(slot_config)

    def _setup_mains(self) -> list[rosys.vision.CalibratableCamera]:
        return [self._build_camera(c) for c in self.main_slot_configs]

    def _build_camera(self, slot_config: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        camera = self._create_camera(slot_config)
        if slot_config.calibration is not None:
            camera.calibration = slot_config.calibration
        self._should_be_connected[camera.id] = slot_config.auto_connect
        return camera

    def _create_camera(self, slot: CameraSlotConfig) -> rosys.vision.CalibratableCamera:
        """Create a camera based on the given slot configuration."""
        camera: rosys.vision.CalibratableCamera
        if rosys.is_simulation():
            camera = SimulatedCalibratableCamera(
                id=slot.camera_id,
                width=slot.width,
                height=slot.height,
                fps=slot.fps,
                color='#cccccc',
                connect_after_init=slot.auto_connect,
            )
        elif isinstance(slot, UsbCameraConfig):
            camera = CalibratableUsbCamera(**slot.camera_kwargs)
        elif isinstance(slot, RtspCameraConfig):
            camera = CalibratableRtspCamera(**slot.camera_kwargs)
        elif isinstance(slot, MjpegCameraConfig):
            camera = CalibratableMjpegCamera(**slot.camera_kwargs)
        else:
            raise ValueError(f'Unknown camera slot type: {type(slot)}')
        self.log.debug('Created %s camera %s', self._camera_config_name(slot), camera.id)
        return camera

    def _camera_config_name(self, config: CameraSlotConfig | None) -> str:
        """Get a human-friendly camera type name based on the config class name, e.g. 'Usb' for UsbCameraConfig."""
        return type(config).__name__.removesuffix('CameraConfig').title() if config else 'Unknown'

    async def update_device_list(self) -> None:
        """Attempt to connect all disconnected cameras that are supposed to be connected."""
        for camera in self.cameras.values():
            if camera.is_connected or not self._should_be_connected[camera.id]:
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

    async def scan(self) -> None:
        """Scan for all available USB, RTSP and MJPEG cameras and log the results."""
        self.log.info('Scanning for cameras...')

        usb_ids = await rosys.run.io_bound(scan_for_connected_devices) or set()
        for uid in sorted(usb_ids):
            self.log.info('USB camera: %s', uid)

        async for mac, ip in find_cameras():
            rtsp_vendor = rtsp_mac_to_vendor(mac)
            mjpeg_vendor = mjpeg_mac_to_vendor(mac)
            types = []
            if rtsp_vendor.name != 'OTHER':
                types.append(f'RTSP ({rtsp_vendor.name})')
            if mjpeg_vendor != MjpegVendorType.OTHER:
                types.append(f'MJPEG ({mjpeg_vendor.name})')
            if types:
                self.log.info('Network camera: mac=%s ip=%s types=%s', mac, ip, ', '.join(types))
            else:
                self.log.debug('Network device: mac=%s ip=%s (unknown vendor)', mac, ip)
        self.log.info('Camera scan complete')

    def developer_ui(self) -> None:
        main_configs = self.main_slot_configs
        slots: list[tuple[str, CalibratableCamera | None, CameraSlotConfig | None]] = []
        if main_configs:
            for i, (cam, cfg) in enumerate(zip(self.mains, main_configs, strict=True)):
                label = 'main' if len(main_configs) == 1 else f'main[{i}]'
                slots.append((label, cam, cfg))
        else:
            slots.append(('main', None, None))
        slots.extend([
            ('front', self.front, self.slot_config('front')),
            ('back', self.back, self.slot_config('back')),
            ('left', self.left, self.slot_config('left')),
            ('right', self.right, self.slot_config('right')),
        ])
        with ui.column():
            ui.label('Cameras').classes('text-center text-bold')
            with ui.grid(columns='auto auto auto auto').classes('items-center'):
                ui.label('Slot').classes('font-bold')
                ui.label('Connected').classes('font-bold')
                ui.label('Resolution').classes('font-bold')
                ui.label('Type').classes('font-bold')
                for name, camera, slot_cfg in slots:
                    ui.label(name)
                    if camera is None:
                        status_bulb()
                        ui.label('—').classes('text-center')
                        ui.label('—').classes('text-center')
                    else:
                        with ui.row().classes('items-center gap-1'):
                            status_bulb().bind_value_from(camera, 'is_connected')
                            self._connection_button(camera.id)
                        resolution = ui.label('—')

                        def update_resolution(label: ui.label = resolution, cam: rosys.vision.CalibratableCamera | None = camera) -> None:
                            if cam is None:
                                return
                            image = cam.latest_captured_image
                            if image is None:
                                label.set_text('—')
                                return
                            text = f'{image.size.width}x{image.size.height}'
                            stream = _stream_resolution(cam)
                            if getattr(cam, 'crop', None) is not None and stream is not None and stream != text:
                                text = f'{stream} \u2192 {text}'
                            label.set_text(text)

                        ui.timer(5.0, update_resolution)
                        ui.label(self._camera_config_name(slot_cfg))
            ui.button('Scan for cameras', on_click=self.scan)

    def _connection_button(self, camera_id: str) -> None:
        """Add a button that connects or disconnects a camera, labeled with the action it offers.

        The icon shows what a click does, so the camera's desired state stays visible next to the
        status bulb: a button offering "Disconnect" on a grey bulb means the connection is failing.

        :param camera_id: Id of the camera to connect or disconnect.
        """
        async def toggle() -> None:
            connected = not self._should_be_connected[camera_id]
            try:
                await self.set_connected(camera_id, connected)
            except Exception as error:
                action = 'connect' if connected else 'disconnect'
                self.log.warning('Failed to %s camera %s', action, camera_id, exc_info=True)
                rosys.notify(f'Failed to {action} camera {camera_id}: {error}', 'negative')

        with ui.button(on_click=toggle).props('flat dense round size=sm') \
                .bind_icon_from(self._should_be_connected, camera_id,
                                backward=lambda connected: 'link_off' if connected else 'link'):
            ui.tooltip().bind_text_from(self._should_be_connected, camera_id,
                                        backward=lambda connected: 'Disconnect' if connected else 'Connect')


def _stream_resolution(camera: rosys.vision.CalibratableCamera) -> str | None:
    """The resolution the camera streams at before cropping, e.g. '2560x1920'."""
    if not isinstance(camera, rosys.vision.ConfigurableCamera):
        return None
    parameters = camera.parameters
    if isinstance(parameters.get('resolution'), tuple):
        width, height = parameters['resolution']
        return f'{width}x{height}'
    if parameters.get('width') and parameters.get('height'):
        return f'{parameters["width"]}x{parameters["height"]}'
    return None
