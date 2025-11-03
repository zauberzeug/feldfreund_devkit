import rosys
from rosys.hardware import (
    BatteryControlHardware,
    BluetoothHardware,
    Bms,
    BmsHardware,
    BmsSimulation,
    Bumper,
    BumperHardware,
    BumperSimulation,
    CanHardware,
    EStop,
    EStopHardware,
    EStopSimulation,
    ExpanderHardware,
    Imu,
    ImuHardware,
    ImuSimulation,
    Robot,
    RobotBrain,
    RobotHardware,
    RobotSimulation,
    SerialCommunication,
    SerialHardware,
    Wheels,
    WheelsSimulation,
)

from ..config import (
    BatteryControlConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    FlashlightMosfetConfiguration,
    ImplementConfiguration,
)
from .can_open_master import CanOpenMasterHardware
from .flashlight import Flashlight, FlashlightHardware, FlashlightHardwareMosfet, FlashlightSimulation
from .implement_hardware import ImplementHardware
from .status_control import StatusControlHardware
from .tracks import TracksHardware, TracksSimulation


class Feldfreund(Robot):
    def __init__(self, config: FeldfreundConfiguration, *,
                 bms: Bms,
                 bumper: Bumper | None,
                 estop: EStop,
                 flashlight: Flashlight | None,
                 implement: ImplementHardware | None,
                 imu: Imu | None,
                 #  safety: Safety | None,
                 wheels: Wheels,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.config = config
        self.bms = bms
        self.bumper = bumper
        self.estop = estop
        self.flashlight = flashlight
        self.implement = implement
        self.imu = imu
        # self.safety = safety
        self.wheels = wheels
        rosys.on_shutdown(self.stop)
        if self.estop:
            self.estop.ESTOP_TRIGGERED.subscribe(self.stop)

    async def stop(self) -> None:
        await self.wheels.stop()
        if self.implement:
            await self.implement.stop()


class FeldfreundHardware(Feldfreund, RobotHardware):
    def __init__(self, config: FeldfreundConfiguration, **kwargs) -> None:
        communication = SerialCommunication()
        robot_brain = RobotBrain(communication,
                                 enable_esp_on_startup=config.robot_brain.enable_esp_on_startup,
                                 use_espresso=config.robot_brain.use_espresso)
        robot_brain.lizard_firmware.flash_params += config.robot_brain.flash_params

        bluetooth = BluetoothHardware(robot_brain, name=config.name)
        serial = SerialHardware(robot_brain)
        expander = ExpanderHardware(robot_brain, serial=serial)
        self.can = CanHardware(robot_brain,
                               expander=expander if config.can.on_expander else None,
                               name=config.can.name,
                               rx_pin=config.can.rx_pin,
                               tx_pin=config.can.tx_pin,
                               baud=config.can.baud)
        estop = EStopHardware(robot_brain, name=config.estop.name, pins=config.estop.pins)
        wheels = TracksHardware(config.tracks, robot_brain, estop, can=self.can)
        can_open_master = CanOpenMasterHardware(robot_brain, can=self.can, name='master')
        bms = BmsHardware(robot_brain,
                          expander=expander if config.bms.on_expander else None,
                          name=config.bms.name,
                          rx_pin=config.bms.rx_pin,
                          tx_pin=config.bms.tx_pin,
                          baud=config.bms.baud,
                          num=config.bms.num,
                          battery_low_threshold=config.bms.battery_low_threshold)
        self.battery_control = self._setup_battery_control(config.battery_control,
                                                           robot_brain=robot_brain,
                                                           bms=bms,
                                                           expander=expander)
        flashlight = self._setup_flashlight(config.flashlight,
                                            robot_brain=robot_brain,
                                            bms=bms,
                                            expander=expander)
        bumper = BumperHardware(robot_brain,
                                expander=expander if config.bumper.on_expander else None,
                                estop=estop,
                                name=config.bumper.name,
                                pins=config.bumper.pins) if config.bumper else None
        imu = ImuHardware(robot_brain,
                          name=config.imu.name,
                          offset_rotation=config.imu.offset_rotation,
                          min_gyro_calibration=config.imu.min_gyro_calibration) if config.imu else None
        self.status_control = StatusControlHardware(robot_brain,
                                                    expander=expander,
                                                    rdyp_pin=39,
                                                    vdp_pin=39) if config.has_status_control else None

        implement = self._setup_implement(config.implement,
                                          robot_brain=robot_brain,
                                          expander=expander,
                                          can=self.can) if config.implement else None
        # safety: SafetyHardware = SafetyHardware(robot_brain, estop=estop, wheels=wheels, bumper=bumper,
        #                                         y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [bluetooth, self.can, wheels, serial, expander, can_open_master,
                   flashlight, bms, estop, self.battery_control, bumper, imu, self.status_control]
        modules = [*modules, *(implement.modules if implement else [])]
        active_modules = [module for module in modules if module is not None]
        super().__init__(bms=bms,
                         bumper=bumper,
                         estop=estop,
                         flashlight=flashlight,
                         implement=implement,
                         imu=imu,
                         #  safety: Safety | None,
                         wheels=wheels,
                         modules=active_modules,
                         robot_brain=robot_brain,
                         **kwargs)

    def _setup_battery_control(self, config: BatteryControlConfiguration, *, robot_brain: RobotBrain, bms: Bms, expander: ExpanderHardware) -> BatteryControlHardware:
        battery_control = BatteryControlHardware(
            robot_brain,
            expander=expander if config.on_expander else None,
            name=config.name,
            reset_pin=config.reset_pin,
            status_pin=config.status_pin,
        )

        async def wait_and_release_battery_relay():
            assert isinstance(battery_control, BatteryControlHardware)
            await rosys.sleep(15)
            # self.log.debug('releasing battery relay on rosys startup')
            await battery_control.release_battery_relay()
        bms.CHARGING_STOPPED.subscribe(battery_control.release_battery_relay)
        rosys.on_startup(wait_and_release_battery_relay)
        return battery_control

    def _setup_flashlight(self, config: FlashlightConfiguration | None, *,
                          robot_brain: RobotBrain,
                          bms: Bms,
                          expander: ExpanderHardware) -> FlashlightHardware | FlashlightHardwareMosfet | None:
        if config is None:
            return None
        if isinstance(config, FlashlightConfiguration):
            return FlashlightHardware(config, robot_brain, expander=expander if config.on_expander else None)
        if isinstance(config, FlashlightMosfetConfiguration):
            return FlashlightHardwareMosfet(config, robot_brain, bms, expander=expander if config.on_expander else None)
        raise NotImplementedError(f'Unknown flashlight configuration: {config}')

    def _setup_implement(self, config: ImplementConfiguration, *,
                         robot_brain: RobotBrain,
                         expander: ExpanderHardware,
                         can: CanHardware) -> ImplementHardware:
        raise NotImplementedError(f'Unknown implement configuration: {config}')


class FeldfreundSimulation(Feldfreund, RobotSimulation):
    def __init__(self, config: FeldfreundConfiguration, *, use_acceleration: bool = False, **kwargs) -> None:
        wheels = TracksSimulation(config.wheels.width) if use_acceleration \
            else WheelsSimulation(config.wheels.width)
        flashlight = FlashlightSimulation() if config.flashlight else None
        estop = EStopSimulation()
        bumper = BumperSimulation(estop=estop) if config.bumper else None
        bms = BmsSimulation(battery_low_threshold=config.bms.battery_low_threshold)
        imu = ImuSimulation(wheels=wheels)
        implement = self._setup_implement(config.implement) if config.implement else None
        # safety = SafetySimulation(wheels=wheels, estop=estop, y_axis=y_axis,
        #                           z_axis=z_axis, flashlight=flashlight)
        modules = [wheels, flashlight, bumper, imu, bms, estop]
        modules = [*modules, *(implement.modules if implement else [])]
        active_modules = [module for module in modules if module is not None]
        super().__init__(config,
                         bms=bms,
                         bumper=bumper,
                         estop=estop,
                         flashlight=flashlight,
                         implement=implement,
                         imu=imu,
                         #  safety: Safety | None,
                         wheels=wheels,
                         modules=active_modules,
                         **kwargs)

    def _setup_implement(self, config: ImplementConfiguration) -> ImplementHardware:
        raise NotImplementedError(f'Unknown implement configuration: {config}')
