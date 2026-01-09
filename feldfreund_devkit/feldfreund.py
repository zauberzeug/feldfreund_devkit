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
    Gnss,
    GnssHardware,
    GnssSimulation,
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

from .config import (
    BatteryControlConfiguration,
    FeldfreundConfiguration,
    FlashlightConfiguration,
    FlashlightMosfetConfiguration,
)
from .hardware import (
    CanOpenMasterHardware,
    Flashlight,
    FlashlightHardware,
    FlashlightHardwareMosfet,
    FlashlightSimulation,
    Safety,
    SafetyHardware,
    SafetyMixin,
    SafetySimulation,
    StatusControlHardware,
    TracksHardware,
    TracksSimulation,
)
from .implement import Implement


class Feldfreund(Robot):
    def __init__(self, config: FeldfreundConfiguration, *,
                 bms: Bms,
                 bumper: Bumper | None,
                 estop: EStop,
                 flashlight: Flashlight | None,
                 imu: Imu | None,
                 safety: Safety,
                 wheels: Wheels,
                 gnss: Gnss | None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.config = config
        self.bms = bms
        self.bumper = bumper
        self.estop = estop
        self.flashlight = flashlight
        self.implement: Implement | None = None
        self.imu = imu
        self.safety = safety
        self.wheels = wheels
        self.gnss = gnss
        self.estop.ESTOP_TRIGGERED.subscribe(self.stop)
        rosys.on_shutdown(self.stop)

    def add_implement(self, implement: Implement) -> None:
        self.implement = implement
        for module in implement.modules:
            self.add_module(module)

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
        self.bluetooth = BluetoothHardware(robot_brain, name=config.bluetooth.name, pin_code=config.bluetooth.pin_code)
        serial = SerialHardware(robot_brain)
        self.expander = ExpanderHardware(robot_brain, serial=serial)
        self.can = CanHardware(robot_brain,
                               expander=self.expander if config.can.on_expander else None,
                               name=config.can.name,
                               rx_pin=config.can.rx_pin,
                               tx_pin=config.can.tx_pin,
                               baud=config.can.baud)
        estop = EStopHardware(robot_brain, name=config.estop.name, pins=config.estop.pins)
        wheels = TracksHardware(config.wheels, robot_brain, estop, can=self.can)
        can_open_master = CanOpenMasterHardware(robot_brain, can=self.can, name='master')
        bms = BmsHardware(robot_brain,
                          expander=self.expander if config.bms.on_expander else None,
                          name=config.bms.name,
                          rx_pin=config.bms.rx_pin,
                          tx_pin=config.bms.tx_pin,
                          baud=config.bms.baud,
                          num=config.bms.num,
                          battery_low_threshold=config.bms.battery_low_threshold)
        self.battery_control = self._setup_battery_control(config.battery_control,
                                                           robot_brain=robot_brain,
                                                           bms=bms,
                                                           expander=self.expander)
        flashlight = self._setup_flashlight(config.flashlight,
                                            robot_brain=robot_brain,
                                            bms=bms,
                                            expander=self.expander)
        bumper = BumperHardware(robot_brain,
                                expander=self.expander if config.bumper.on_expander else None,
                                estop=estop,
                                name=config.bumper.name,
                                pins=config.bumper.pins) if config.bumper else None
        imu = ImuHardware(robot_brain,
                          name=config.imu.name,
                          offset_rotation=config.imu.offset_rotation,
                          min_gyro_calibration=config.imu.min_gyro_calibration) if config.imu else None
        self.safety: SafetyHardware = SafetyHardware(robot_brain, estop=estop, wheels=wheels, bumper=bumper)
        if flashlight:
            self.safety.add_module(flashlight)
        self.status_control = StatusControlHardware(robot_brain, expander=self.expander, rdyp_pin=39, vdp_pin=39)
        gnss = GnssHardware(antenna_pose=config.gnss.pose) if config.gnss else None
        modules = [self.bluetooth, self.can, wheels, serial, self.expander, can_open_master,
                   flashlight, bms, estop, self.battery_control, bumper, imu, self.safety, self.status_control]
        active_modules = [module for module in modules if module is not None]
        super().__init__(config,
                         bms=bms,
                         bumper=bumper,
                         estop=estop,
                         flashlight=flashlight,
                         imu=imu,
                         safety=self.safety,
                         wheels=wheels,
                         gnss=gnss,
                         modules=active_modules,
                         robot_brain=robot_brain,
                         **kwargs)

    def add_implement(self, implement: Implement) -> None:
        super().add_implement(implement)
        if isinstance(implement, SafetyMixin):
            self.add_safety_module(implement)
        for module in implement.modules:
            if isinstance(module, SafetyMixin):
                self.add_safety_module(module)

    def add_safety_module(self, module: SafetyMixin) -> None:
        self.safety.add_module(module)
        self.robot_brain.lizard_code = self.generate_lizard_code()

    def generate_lizard_code(self) -> str:
        # TODO: This is a hack to move safety and status control to the end of the lizard code.
        self.modules.remove(self.safety)
        self.modules.remove(self.status_control)
        self.modules.append(self.safety)
        self.modules.append(self.status_control)
        return super().generate_lizard_code()

    def _setup_battery_control(self, config: BatteryControlConfiguration, *,
                               robot_brain: RobotBrain,
                               bms: Bms,
                               expander: ExpanderHardware | None) -> BatteryControlHardware:
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

    def _setup_flashlight(self, config: FlashlightConfiguration | FlashlightMosfetConfiguration | None, *,
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


class FeldfreundSimulation(Feldfreund, RobotSimulation):
    def __init__(self, config: FeldfreundConfiguration, *, use_acceleration: bool = False, **kwargs) -> None:
        wheels = TracksSimulation(config.wheels.width) if use_acceleration \
            else WheelsSimulation(config.wheels.width)
        flashlight = FlashlightSimulation() if config.flashlight else None
        estop = EStopSimulation()
        bumper = BumperSimulation(estop=estop) if config.bumper else None
        bms = BmsSimulation(battery_low_threshold=config.bms.battery_low_threshold)
        imu = ImuSimulation(wheels=wheels)
        safety = SafetySimulation(wheels=wheels, estop=estop, bumper=bumper)
        # NOTE: quick fix for https://github.com/zauberzeug/feldfreund/issues/348
        gnss = GnssSimulation(wheels=wheels, lat_std_dev=1e-10, lon_std_dev=1e-10, heading_std_dev=1e-10) if rosys.is_test \
            else GnssSimulation(wheels=wheels, lat_std_dev=0.008, lon_std_dev=0.008, heading_std_dev=0.01, interval=0.1, latency=0.1)
        modules = [wheels, flashlight, bumper, imu, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(config,
                         bms=bms,
                         bumper=bumper,
                         estop=estop,
                         flashlight=flashlight,
                         imu=imu,
                         safety=safety,
                         wheels=wheels,
                         gnss=gnss,
                         modules=active_modules,
                         **kwargs)
