import rosys
from rosys.geometry import PoseStep, Velocity
from rosys.hardware import WheelsSimulation


class TracksSimulation(WheelsSimulation):
    def __init__(self, width: float = 0.5, *, linear_acceleration: float = 2.0, linear_deceleration: float = 0.5) -> None:
        """Simulate differential drive wheels with acceleration and deceleration handling.

        :param width: The distance between the wheels in meters.
        :param linear_acceleration: The maximum linear acceleration rate in m/s².
        :param linear_deceleration: The maximum linear deceleration rate in m/s².
        """
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
