import rosys
from rosys.geometry import Velocity
from rosys.hardware import CanHardware, EStopHardware, ModuleHardware, RobotBrain, Wheels
from rosys.helpers import remove_indentation

from ...config import TracksConfiguration


class TracksHardware(Wheels, ModuleHardware):
    """Expands the RoSys wheels hardware to control the field friend's tracked wheels with dual motors."""
    MAX_VALID_LINEAR_VELOCITY = 3.0
    MAX_VALID_ANGULAR_VELOCITY = 3.5
    ERROR_FLAG_VERSION = 6

    def __init__(self, config: TracksConfiguration,
                 robot_brain: RobotBrain,
                 estop: EStopHardware, *,
                 can: CanHardware,
                 m_per_tick: float = 0.01,
                 width: float = 0.5) -> None:
        self.config = config
        self.estop = estop
        self._l0_error = False
        self._r0_error = False
        self._l1_error = False
        self._r1_error = False
        lizard_code = remove_indentation(f'''
            l0 = ODriveMotor({can.name}, {config.left_back_can_address}{', 6' if config.odrive_version == self.ERROR_FLAG_VERSION else ''})
            r0 = ODriveMotor({can.name}, {config.right_back_can_address}{', 6' if config.odrive_version == self.ERROR_FLAG_VERSION else ''})
            l1 = ODriveMotor({can.name}, {config.left_front_can_address}{', 6' if config.odrive_version == self.ERROR_FLAG_VERSION else ''})
            r1 = ODriveMotor({can.name}, {config.right_front_can_address}{', 6' if config.odrive_version == self.ERROR_FLAG_VERSION else ''})
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
            {config.name}.width = {width}
            {config.name}_front.width = {width}
            {config.name}.shadow({config.name}_front)
        ''')
        core_message_fields = [f'{config.name}.linear_speed:3', f'{config.name}.angular_speed:3']
        if config.odrive_version == self.ERROR_FLAG_VERSION:
            core_message_fields.extend(['l0.motor_error_flag', 'r0.motor_error_flag',
                                       'l1.motor_error_flag', 'r1.motor_error_flag'])
        else:
            self.log.warning('ODrive firmware is deprecated. Please update to benefit from the motor error detection.')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

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
        if self.estop.active:
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
        if self.config.odrive_version != self.ERROR_FLAG_VERSION:
            return
        motor_error = any([self._l0_error, self._r0_error, self._l1_error, self._r1_error])
        self._l0_error = int(words.pop(0)) == 1
        self._r0_error = int(words.pop(0)) == 1
        self._l1_error = int(words.pop(0)) == 1
        self._r1_error = int(words.pop(0)) == 1
        if self.motor_error and not motor_error:
            rosys.notify('Motor Error', 'negative')

    def developer_ui(self) -> None:
        pass
