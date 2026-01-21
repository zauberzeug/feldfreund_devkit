import rosys
from nicegui import ui
from rosys.geometry import PoseStep, Velocity
from rosys.hardware import CanHardware, EStopHardware, ModuleHardware, RobotBrain, Wheels, WheelsSimulation
from rosys.helpers import remove_indentation

from ..config import TracksConfiguration


class TracksHardware(Wheels, ModuleHardware):
    """Expands the RoSys wheels hardware to control the field friend's tracked wheels with dual motors."""
    MAX_VALID_LINEAR_VELOCITY = 3.0
    MAX_VALID_ANGULAR_VELOCITY = 3.5
    ERROR_FLAG_VERSION = 6

    def __init__(self, config: TracksConfiguration,
                 robot_brain: RobotBrain,
                 estop: EStopHardware, *,
                 can: CanHardware) -> None:
        self.config = config
        self._estop = estop
        self._l0_error = False
        self._r0_error = False
        self._l1_error = False
        self._r1_error = False
        self._l0_temperature = 0.0
        self._r0_temperature = 0.0
        self._l1_temperature = 0.0
        self._r1_temperature = 0.0
        m_per_tick = self.config.m_per_tick
        version_suffix = f', {self.ERROR_FLAG_VERSION}' if config.odrive_version == self.ERROR_FLAG_VERSION else ''
        lizard_code = remove_indentation(f'''
            l0 = ODriveMotor({can.name}, {config.left_back_can_address}{version_suffix})
            r0 = ODriveMotor({can.name}, {config.right_back_can_address}{version_suffix})
            l1 = ODriveMotor({can.name}, {config.left_front_can_address}{version_suffix})
            r1 = ODriveMotor({can.name}, {config.right_front_can_address}{version_suffix})
            l0.m_per_tick = {m_per_tick}
            r0.m_per_tick = {m_per_tick}
            l1.m_per_tick = {m_per_tick}
            r1.m_per_tick = {m_per_tick}
            l0.reversed = {'true' if config.is_left_reversed else 'false'}
            r0.reversed = {'true' if config.is_right_reversed else 'false'}
            l1.reversed = {'true' if config.is_left_reversed else 'false'}
            r1.reversed = {'true' if config.is_right_reversed else 'false'}
            {config.name} = ODriveWheels(l0, r0)
            {config.name}_front = ODriveWheels(l1, r1)
            {config.name}.width = {self.config.width}
            {config.name}_front.width = {self.config.width}
            {config.name}.shadow({config.name}_front)
        ''')
        core_message_fields = [f'{config.name}.linear_speed:3', f'{config.name}.angular_speed:3']
        if config.has_temperature_sensor:
            core_message_fields.extend(['l0.motor_temperature', 'r0.motor_temperature',
                                        'l1.motor_temperature', 'r1.motor_temperature'])
        if config.odrive_version == self.ERROR_FLAG_VERSION:
            core_message_fields.extend(['l0.motor_error_flag', 'r0.motor_error_flag',
                                       'l1.motor_error_flag', 'r1.motor_error_flag'])
        else:
            self.log.warning('ODrive firmware is deprecated. Please update to benefit from the motor error detection.')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def motor_error(self) -> bool:
        if self.config.odrive_version != self.ERROR_FLAG_VERSION:
            self.log.warning('Motor error detection is not available for this ODrive firmware version.')
            return False
        return any([self._l0_error, self._r0_error, self._l1_error, self._r1_error])

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        if linear == 0.0:
            linear = -0.0
        if angular == 0.0:
            angular = -0.0  # TODO: Temp fix
        if not self.robot_brain.is_ready:
            self.log.warning('Robot brain not ready')
            return
        await self.robot_brain.send(f'{self.config.name}.speed({linear}, {angular})')

    async def reset_motors(self) -> None:
        if self._estop.active:
            return
        if not self.motor_error:
            return
        if self._l0_error:
            await self.robot_brain.send('l0.reset_motor()')
        if self._r0_error:
            await self.robot_brain.send('r0.reset_motor()')
        if self._l1_error:
            await self.robot_brain.send('l1.reset_motor()')
        if self._r1_error:
            await self.robot_brain.send('r1.reset_motor()')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        if abs(velocity.linear) <= self.MAX_VALID_LINEAR_VELOCITY and abs(velocity.angular) <= self.MAX_VALID_ANGULAR_VELOCITY:
            self.VELOCITY_MEASURED.emit([velocity])
        else:
            self.log.error('Velocity is too high: (%s, %s)', velocity.linear, velocity.angular)
        if self.config.has_temperature_sensor:
            self._l0_temperature = float(words.pop(0))
            self._r0_temperature = float(words.pop(0))
            self._l1_temperature = float(words.pop(0))
            self._r1_temperature = float(words.pop(0))
        if self.config.odrive_version == self.ERROR_FLAG_VERSION:
            motor_error = any([self._l0_error, self._r0_error, self._l1_error, self._r1_error])
            self._l0_error = int(words.pop(0)) == 1
            self._r0_error = int(words.pop(0)) == 1
            self._l1_error = int(words.pop(0)) == 1
            self._r1_error = int(words.pop(0)) == 1
            if self.motor_error and not motor_error:
                rosys.notify('Motor Error', 'negative')

    def developer_ui(self) -> None:
        @ui.refreshable
        def _ui() -> None:
            ui.label('ODrive Motor Errors').classes('text-center text-bold')
            if self.config.odrive_version == self.ERROR_FLAG_VERSION:
                with ui.grid(columns=2).classes('gap-0'):
                    ui.label(f'L0: {"Error" if self._l0_error else "No error"}')
                    ui.label(f'L1: {"Error" if self._l1_error else "No error"}')
                    ui.label(f'R0: {"Error" if self._r0_error else "No error"}')
                    ui.label(f'R1: {"Error" if self._r1_error else "No error"}')
                ui.button('Reset motor errors', on_click=self.reset_motors).set_enabled(self.motor_error)
            if self.config.has_temperature_sensor:
                with ui.grid(columns=2).classes('gap-0'):
                    ui.label(f'L0: {self._l0_temperature:.1f}°C')
                    ui.label(f'L1: {self._l1_temperature:.1f}°C')
                    ui.label(f'R0: {self._r0_temperature:.1f}°C')
                    ui.label(f'R1: {self._r1_temperature:.1f}°C')
        _ui()
        ui.timer(rosys.config.ui_update_interval, _ui.refresh)


class TracksSimulation(WheelsSimulation):
    """Simulated tracks with acceleration and deceleration handling."""

    def __init__(self, width: float = 0.5, *, linear_acceleration: float = 2.0, linear_deceleration: float = 0.5) -> None:
        super().__init__(width)

        self.linear_acceleration: float = linear_acceleration
        """The maximum linear acceleration rate."""

        self.linear_deceleration: float = linear_deceleration
        """The maximum linear deceleration rate."""

    @property
    def angular_acceleration(self) -> float:
        """Calculate angular acceleration from linear acceleration using differential drive kinematics."""
        return 2 * self.linear_acceleration / self.width

    @property
    def angular_deceleration(self) -> float:
        """Calculate angular deceleration from linear deceleration using differential drive kinematics."""
        return 2 * self.linear_deceleration / self.width

    async def drive(self, linear: float, angular: float) -> None:
        self.linear_target_speed = linear
        self.angular_target_speed = angular

    async def step(self, dt: float) -> None:
        if self.is_blocking:
            self.linear_velocity = 0
            self.angular_velocity = 0
            self.linear_target_speed = 0
            self.angular_target_speed = 0
        else:
            if self.linear_velocity < self.linear_target_speed:
                self.linear_velocity = min(self.linear_velocity + self.linear_acceleration * dt,
                                           self.linear_target_speed)
            elif self.linear_velocity > self.linear_target_speed:
                self.linear_velocity = max(self.linear_velocity - self.linear_deceleration * dt,
                                           self.linear_target_speed)
            if self.angular_velocity < self.angular_target_speed:
                self.angular_velocity = min(self.angular_velocity + self.angular_acceleration * dt,
                                            self.angular_target_speed)
            elif self.angular_velocity > self.angular_target_speed:
                self.angular_velocity = max(self.angular_velocity - self.angular_deceleration * dt,
                                            self.angular_target_speed)

        self.linear_velocity *= 1 - self.friction_factor
        self.angular_velocity *= 1 - self.friction_factor
        left_speed = self.linear_velocity - self.angular_velocity * self.width / 2
        right_speed = self.linear_velocity + self.angular_velocity * self.width / 2
        left_speed *= 1 - self.slip_factor_left
        right_speed *= 1 - self.slip_factor_right
        self.pose += PoseStep(linear=dt * (left_speed + right_speed) / 2,
                              angular=dt * (right_speed - left_speed) / self.width,
                              time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity,
                            angular=self.angular_velocity, time=self.pose.time)
        self.VELOCITY_MEASURED.emit([velocity])
