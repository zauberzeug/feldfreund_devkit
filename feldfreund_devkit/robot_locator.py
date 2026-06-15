import logging
from collections import deque
from typing import Any

import numpy as np
import rosys
import rosys.helpers
from nicegui import Event, ui
from rosys.driving import PoseProvider, VelocityProvider
from rosys.geometry import Frame3d, FrameProvider, Pose, Pose3d, Rotation, Velocity
from rosys.hardware import Gnss, GnssMeasurement, GnssSimulation, Imu, ImuMeasurement, Wheels, WheelsSimulation

from .config import GnssConfiguration


class RobotLocator(rosys.persistence.Persistable, FrameProvider, PoseProvider, VelocityProvider):
    """Extended Kalman filter for robot pose estimation using odometry, GNSS, and IMU."""

    R_ODOM_LINEAR = 0.1
    R_ODOM_ANGULAR = 0.097
    R_IMU_ANGULAR = 0.01
    ODOMETRY_ANGULAR_WEIGHT = 0.1
    POSE_HISTORY_DURATION = 2.0
    """Seconds of past pose estimates kept for time-based lookups via :meth:`_state_at`."""
    VELOCITY_SMOOTHING_DURATION = 0.2
    """Window over which the filtered pose is differenced into the emitted ``VELOCITY_MEASURED`` velocity."""
    VELOCITY_GAP_THRESHOLD = 0.5
    """A gap between consecutive pose estimates above this (e.g. a standstill, during which no history is
    recorded) is treated as a discontinuity the velocity is not differenced across. Kept separate from
    ``VELOCITY_SMOOTHING_DURATION`` so tuning the smoothing window can't make a normal sample interval look
    like a gap; must stay comfortably above the odometry sample interval (else every step reads as a gap)."""

    def __init__(self,
                 wheels: Wheels, *,
                 gnss: Gnss | None = None,
                 imu: Imu | None = None,
                 gnss_config: GnssConfiguration | None = None) -> None:
        """Robot Locator based on an extended Kalman filter."""
        super().__init__()
        self.log = logging.getLogger('feldfreund.robot_locator')

        self._wheels = wheels
        self._gnss = gnss
        self._imu = imu
        self._gnss_config = gnss_config

        state_size = 3
        self._x: np.ndarray = np.zeros((state_size, 1))
        self._Sxx: np.ndarray = np.zeros((state_size, state_size))
        self._pose_timestamp = rosys.time()
        # NOTE: the prediction step needs to be run once before the first GNSS update
        self._first_prediction_done = False

        self._pose = Pose(x=0.0, y=0.0, yaw=0.0, time=self._pose_timestamp)
        # ring buffer of (timestamp, state, covariance) for time-based pose lookups
        self._pose_history: deque[tuple[float, np.ndarray, np.ndarray]] = deque()
        self.POSE_UPDATED: Event[Pose] = Event()
        """Emitted when the pose has been updated (argument: current ``Pose``)."""

        self._pose_frame = Pose3d().as_frame('feldfreund.robot_locator')
        self.FRAME_UPDATED: Event[Frame3d] = Event()
        """Emitted when the pose frame has been updated (argument: the new pose frame)."""

        self.VELOCITY_MEASURED: Event[list[Velocity]] = Event()
        """Emitted per processed wheel velocity with the filtered velocity (argument: list of ``Velocity``)."""
        self._velocity = Velocity(linear=0.0, angular=0.0, time=self._pose_timestamp)
        """Last emitted filtered velocity, kept for the developer UI."""

        self._ignore_gnss = gnss is None
        self._ignore_imu = imu is None
        self._auto_tilt_correction = True
        self._r_odom_linear = self.R_ODOM_LINEAR
        self._r_odom_angular = self.R_ODOM_ANGULAR
        self._r_imu_angular = self.R_IMU_ANGULAR
        self._odometry_angular_weight = self.ODOMETRY_ANGULAR_WEIGHT

        self._previous_imu_measurement: ImuMeasurement | None = None

        self._wheels.VELOCITY_MEASURED.subscribe(self._handle_velocity_measurement)
        if self._gnss is not None:
            self._gnss.NEW_MEASUREMENT.subscribe(self._handle_gnss_measurement)
        rosys.on_startup(self.reset)

    @property
    def frame(self) -> Frame3d:
        return self._pose_frame

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def prediction(self) -> Pose:
        return self.pose

    @property
    def uncertainty(self) -> tuple[float, float, float]:
        return self._Sxx[0, 0], self._Sxx[1, 1], self._Sxx[2, 2]

    def _state_at(self, time: float) -> tuple[Pose, np.ndarray]:
        """Return the estimated pose and covariance at the given ``time``, interpolated from the history.

        Useful to project time-stamped measurements (e.g. camera detections) or to re-apply a latent
        GNSS update with the filter state as it was when the measurement was taken, instead of the live
        state. A single pass over the history yields both pose and covariance. Outside the buffered window
        the result is clamped to the oldest/newest available estimate. The returned ``Pose`` is a fresh
        object stamped with the requested ``time`` and the covariance is a fresh array; neither aliases
        the live internal state.
        """
        if not self._pose_history:
            return Pose(x=self._pose.x, y=self._pose.y, yaw=self._pose.yaw, time=time), self._Sxx.copy()
        if time >= self._pose_history[-1][0]:
            _, state, covariance = self._pose_history[-1]
            return self._pose_from_state(state, time), covariance.copy()
        oldest_time, oldest_state, oldest_covariance = self._pose_history[0]
        if time <= oldest_time:
            return self._pose_from_state(oldest_state, time), oldest_covariance.copy()
        previous_time, previous_state, previous_covariance = oldest_time, oldest_state, oldest_covariance
        for current_time, current_state, current_covariance in self._pose_history:
            if previous_time <= time <= current_time:
                span = current_time - previous_time
                alpha = (time - previous_time) / span if span > 0 else 0.0
                x = previous_state[0, 0] + alpha * (current_state[0, 0] - previous_state[0, 0])
                y = previous_state[1, 0] + alpha * (current_state[1, 0] - previous_state[1, 0])
                yaw = previous_state[2, 0] + alpha * rosys.helpers.angle(previous_state[2, 0], current_state[2, 0])
                # convex combination of two PSD matrices stays PSD
                covariance = previous_covariance + alpha * (current_covariance - previous_covariance)
                return Pose(x=x, y=y, yaw=yaw, time=time), covariance
            previous_time, previous_state, previous_covariance = current_time, current_state, current_covariance
        _, state, covariance = self._pose_history[-1]  # unreachable: time is within the window
        return self._pose_from_state(state, time), covariance.copy()

    @staticmethod
    def _pose_from_state(state: np.ndarray, time: float) -> Pose:
        return Pose(x=state[0, 0], y=state[1, 0], yaw=state[2, 0], time=time)

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'r_odom_linear': self._r_odom_linear,
            'r_odom_angular': self._r_odom_angular,
            'r_imu_angular': self._r_imu_angular,
            'odometry_angular_weight': self._odometry_angular_weight,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self._r_odom_linear = data.get('r_odom_linear', self.R_ODOM_LINEAR)
        self._r_odom_angular = data.get('r_odom_angular', self.R_ODOM_ANGULAR)
        self._r_imu_angular = data.get('r_imu_angular', self.R_IMU_ANGULAR)
        self._odometry_angular_weight = data.get('odometry_angular_weight', self.ODOMETRY_ANGULAR_WEIGHT)

    async def _handle_velocity_measurement(self, velocities: list[Velocity]) -> None:
        """Implements the 'prediction' step of the Kalman filter."""
        for velocity in velocities:
            dt = velocity.time - self._pose_timestamp
            self._pose_timestamp = velocity.time
            if (not self._first_prediction_done) and (self._imu is not None):
                self._previous_imu_measurement = self._imu.last_measurement

            if velocity.linear == 0 and velocity.angular == 0 and self._first_prediction_done:
                # NOTE: The robot is not moving, so we don't need to update the state
                self._emit_velocity(Velocity(linear=0.0, angular=0.0, time=velocity.time))
                continue

            v = velocity.linear
            omega = velocity.angular
            r_angular = self._r_odom_angular
            if not self._ignore_imu and self._imu is not None:
                imu_omega = self._get_imu_angular_velocity()
                if imu_omega is not None:
                    v, omega = self._combine_odom_imu(v, omega, imu_omega)
                    r_angular = self._odometry_angular_weight * self._r_odom_angular + \
                        (1 - self._odometry_angular_weight) * self._r_imu_angular

            theta = self._x[2, 0]
            theta_new = theta + omega * dt
            theta_avg = (theta + theta_new) / 2  # Average orientation

            self._x[0, 0] += v * np.cos(theta_avg) * dt
            self._x[1, 0] += v * np.sin(theta_avg) * dt
            self._x[2, 0] = theta_new

            F = np.array([
                [1, 0, -v * np.sin(theta_avg) * dt],
                [0, 1, v * np.cos(theta_avg) * dt],
                [0, 0, 1],
            ])

            R = np.array([
                [(self._r_odom_linear * dt * np.cos(theta_avg))**2, 0, 0],
                [0, (self._r_odom_linear * dt * np.sin(theta_avg))**2, 0],
                [0, 0, (r_angular * dt)**2]
            ])
            self._Sxx = F @ self._Sxx @ F.T + R
            self._update_frame()
            self._first_prediction_done = True
            self._emit_velocity(self._estimate_velocity(velocity.time))

    def _emit_velocity(self, velocity: Velocity) -> None:
        self._velocity = velocity
        self.VELOCITY_MEASURED.emit([velocity])

    def _estimate_velocity(self, time: float) -> Velocity:
        """Estimate a smoothed velocity by differencing the filtered pose over a short window.

        Differencing the EKF pose (instead of forwarding the raw wheel speed) yields a velocity that
        reflects the IMU blend and GNSS corrections folded into the state. The window spreads a discrete
        GNSS correction jump over several samples rather than emitting it as a single-step spike.

        The walk back stops at the first gap larger than ``VELOCITY_GAP_THRESHOLD`` (e.g. a standstill,
        during which no history is recorded), so velocity is never differenced across the gap; it recovers
        one sample later.
        """
        newest_time, newest_x, _ = self._pose_history[-1]
        target_time = newest_time - self.VELOCITY_SMOOTHING_DURATION
        earlier_time, earlier_x = newest_time, newest_x
        previous_time = newest_time
        for entry_time, entry_x, _ in reversed(self._pose_history):
            if previous_time - entry_time > self.VELOCITY_GAP_THRESHOLD:
                break  # consecutive gap exceeds the threshold (e.g. a standstill) — don't difference across it
            earlier_time, earlier_x, previous_time = entry_time, entry_x, entry_time
            if entry_time <= target_time:
                break
        dt = newest_time - earlier_time
        if dt <= 0:
            return Velocity(linear=0.0, angular=0.0, time=time)
        dx = newest_x[0, 0] - earlier_x[0, 0]
        dy = newest_x[1, 0] - earlier_x[1, 0]
        dyaw = rosys.helpers.angle(earlier_x[2, 0], newest_x[2, 0])
        direction = earlier_x[2, 0] + dyaw / 2  # average heading over the window for the forward projection
        linear = (dx * np.cos(direction) + dy * np.sin(direction)) / dt
        angular = dyaw / dt
        return Velocity(linear=linear, angular=angular, time=time)

    def _get_imu_angular_velocity(self) -> float | None:
        if self._previous_imu_measurement is None or self._imu is None or self._imu.last_measurement is None:
            return None
        new_imu_measurement = self._imu.last_measurement
        imu_dtheta = rosys.helpers.angle(self._previous_imu_measurement.rotation.yaw,
                                         new_imu_measurement.rotation.yaw)
        imu_dt = new_imu_measurement.time - self._previous_imu_measurement.time
        imu_angular_velocity = imu_dtheta / imu_dt if imu_dt > 0 else None
        self._previous_imu_measurement = new_imu_measurement
        return imu_angular_velocity

    def _combine_odom_imu(self, odometry_v: float, odometry_omega: float, imu_omega: float) -> tuple[float, float]:
        """Linear combination of odometry and IMU angular velocity.
        The linear velocity is adjusted based on the angular velocity difference (indicating a slip)."""
        v = odometry_v
        ow = self._odometry_angular_weight
        omega = ow * odometry_omega + (1 - ow) * imu_omega
        # Adjust linear velocity based on angular velocity difference
        angular_difference = abs(odometry_omega - imu_omega)
        correction_factor = 1.0 / (1.0 + angular_difference)
        v = ow * v + (1.0 - ow) * (v * correction_factor)
        return v, omega

    def _handle_gnss_measurement(self, gnss_measurement: GnssMeasurement) -> None:
        """Triggers the 'update' step of the Kalman filter."""
        if self._ignore_gnss:
            return
        if not np.isfinite(gnss_measurement.heading_std_dev):
            # normally we would only handle the position if no heading is available,
            # but the Feldfreund needs the rtk accuracy to function properly
            return
        pose, r_xy, r_theta = self._get_local_pose_and_uncertainty(gnss_measurement)
        if self._auto_tilt_correction and isinstance(self._imu, Imu) and not self._ignore_imu and self._imu.last_measurement is not None:
            pose = self._correct_gnss_with_imu(pose)
        # The measurement reflects the pose at the moment of the fix, so compare it against the
        # estimate from that time instead of the live pose. Fusing against the live pose would pull
        # the state backwards by ~v*latency along the driving direction (out-of-sequence update).
        # The gain is computed from the current covariance, which barely changes over the latency.
        # On hardware ``measurement.time`` is the arrival time (≈ now), while the true latency lives
        # in ``measurement.age`` (fix epoch - now); ``now + age`` recovers the fix epoch on both
        # hardware and simulation and adapts to each system's (variable) latency without a constant.
        fix_time = rosys.time() + gnss_measurement.age
        reference, _ = self._state_at(fix_time)
        yaw_measurement = reference.yaw + rosys.helpers.angle(reference.yaw, pose.yaw)
        z = [[pose.x], [pose.y], [yaw_measurement]]
        h = [[reference.x], [reference.y], [reference.yaw]]
        H = np.eye(3)
        variance = np.array([r_xy, r_xy, r_theta], dtype=np.float64)**2
        Q = np.diag(variance)
        self._update(z=np.array(z), h=np.array(h), H=H, Q=Q)

    def _get_local_pose_and_uncertainty(self, gnss_measurement: GnssMeasurement) -> tuple[Pose, float, float]:
        pose = gnss_measurement.pose.to_local()
        pose.yaw = self._x[2, 0] + rosys.helpers.angle(self._x[2, 0], pose.yaw)
        r_xy = (gnss_measurement.latitude_std_dev + gnss_measurement.longitude_std_dev) / 2
        r_theta = np.deg2rad(gnss_measurement.heading_std_dev)
        return pose, r_xy, r_theta

    def _correct_gnss_with_imu(self, pose: Pose) -> Pose:
        assert isinstance(self._imu, Imu)
        assert self._imu.last_measurement is not None
        assert self._gnss_config is not None
        roll = self._imu.last_measurement.rotation.roll
        pitch = self._imu.last_measurement.rotation.pitch
        antenna_roll_correction = Pose(x=0, y=self._gnss_config.y * (1 - np.cos(roll)), yaw=0)
        height_correction = Pose(x=self._gnss_config.z * np.sin(-pitch),
                                 y=self._gnss_config.z * np.sin(roll), yaw=0)
        return pose.transform_pose(antenna_roll_correction).transform_pose(height_correction)

    def _update(self, *, z: np.ndarray, h: np.ndarray, H: np.ndarray, Q: np.ndarray) -> None:
        S = H @ self._Sxx @ H.T + Q
        # Use Cholesky decomposition for numerical stability
        try:
            L = np.linalg.cholesky(S)
            K = self._Sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        except np.linalg.LinAlgError:
            S += np.eye(S.shape[0]) * 1e-6
            L = np.linalg.cholesky(S)
            K = self._Sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        self._x = self._x + K @ (z - h)
        # Joseph form keeps the covariance symmetric and positive semi-definite, unlike the
        # naive (I - K H) P which drifts into asymmetry and negative eigenvalues (#348, #372)
        A = np.eye(self._Sxx.shape[0]) - K @ H
        self._Sxx = A @ self._Sxx @ A.T + K @ Q @ K.T
        self._Sxx = (self._Sxx + self._Sxx.T) / 2  # remove residual asymmetry from floating-point error
        self._update_frame()

    def _update_frame(self) -> None:
        self._pose_frame.x = self._x[0, 0]
        self._pose_frame.y = self._x[1, 0]
        self._pose_frame.rotation = Rotation.from_euler(0, 0, self._x[2, 0])
        self._pose.x = self._x[0, 0]
        self._pose.y = self._x[1, 0]
        self._pose.yaw = self._x[2, 0]
        self._pose.time = self._pose_timestamp
        self._record_pose_history()
        self.FRAME_UPDATED.emit(self._pose_frame)
        self.POSE_UPDATED.emit(self._pose)

    def _record_pose_history(self) -> None:
        entry = (self._pose_timestamp, self._x.copy(), self._Sxx.copy())
        # predict and the following GNSS update share a timestamp; keep only the corrected state
        # (both come from the same _pose_timestamp value, so exact float equality is intentional)
        if self._pose_history and self._pose_history[-1][0] == self._pose_timestamp:
            self._pose_history[-1] = entry
        else:
            self._pose_history.append(entry)
        cutoff = self._pose_timestamp - self.POSE_HISTORY_DURATION
        while len(self._pose_history) > 1 and self._pose_history[0][0] < cutoff:
            self._pose_history.popleft()

    async def reset(self, *, gnss_timeout: float = 2.0) -> None:
        reset_pose = Pose(x=0.0, y=0.0, yaw=0.0, time=rosys.time())
        r_xy = 0.0
        r_theta = 0.0
        if isinstance(self._wheels, WheelsSimulation):
            self._wheels.pose = reset_pose
        if self._gnss is not None and not self._ignore_gnss:
            try:
                # pylint: disable=protected-access
                if isinstance(self._gnss, GnssSimulation):
                    last_latency = self._gnss._latency
                    self._gnss._latency = 0.0
                await self._gnss.NEW_MEASUREMENT.emitted(gnss_timeout)
                assert self._gnss.last_measurement is not None
                reset_pose, r_xy, r_theta = self._get_local_pose_and_uncertainty(self._gnss.last_measurement)
                if isinstance(self._gnss, GnssSimulation):
                    self._gnss._latency = last_latency
                # pylint: enable=protected-access
            except TimeoutError:
                self.log.error('GNSS timeout while resetting position. Activate _ignore_gnss to use zero position.')
                return
            except AssertionError:
                self.log.error('''GNSS measurement is not available while resetting position.
                               Activate _ignore_gnss to use zero position.''')
                return
        self._reset(x=reset_pose.x, y=reset_pose.y, yaw=reset_pose.yaw, r_xy=r_xy, r_theta=r_theta)

    def _reset(self, *, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, r_xy: float = 0.0, r_theta: float = 0.0) -> None:
        self._x[:, 0] = [x, y, yaw]
        variance = np.array([r_xy, r_xy, r_theta], dtype=np.float64)**2
        self._Sxx.fill(0)
        np.fill_diagonal(self._Sxx, variance)
        self._pose_timestamp = rosys.time()
        self._pose_history.clear()  # the pose jumps discontinuously, so past estimates are invalid
        self._update_frame()

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Kalman Filter').classes('text-center text-bold')
            with ui.grid(columns=2).classes('w-full gap-0'):
                ui.label().bind_text_from(self, 'pose', lambda p: f'x: {p.x:.3f}m')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {m[0, 0]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'y: {p.y:.3f}m')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {m[1, 1]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'θ: {p.yaw_deg:.2f}°')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {np.rad2deg(m[2, 2]):.2f}°')
                ui.label().bind_text_from(self, '_velocity', lambda v: f'v: {v.linear:.2f}m/s')
                ui.label().bind_text_from(self, '_velocity', lambda v: f'ω: {np.rad2deg(v.angular):.1f}°/s')

            with ui.grid(columns=2).classes('w-full'):
                ui.checkbox('Ignore GNSS', value=self._ignore_gnss).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, '_ignore_gnss').tooltip('Ignore GNSS measurements. When deactivated, reset the filter for better positioning.')
                ui.checkbox('Ignore IMU', value=self._ignore_imu).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, '_ignore_imu')
                ui.checkbox('Correct GNSS with IMU', value=self._auto_tilt_correction).props('dense').classes('col-span-2') \
                    .bind_value_to(self, '_auto_tilt_correction')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R v linear', min=0, step=0.01, format='%.3f', suffix='m/s', value=self._r_odom_linear, on_change=self.request_backup) \
                        .bind_value_to(self, '_r_odom_linear')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω odom', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self._r_odom_angular), on_change=self.request_backup) \
                        .bind_value_to(self, '_r_odom_angular', forward=np.deg2rad)  # type: ignore[arg-type]
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω imu', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self._r_imu_angular), on_change=self.request_backup) \
                        .bind_value_to(self, '_r_imu_angular', forward=np.deg2rad)  # type: ignore[arg-type]
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='ω odom weight', min=0, step=0.01, format='%.3f', value=self._odometry_angular_weight, on_change=self.request_backup) \
                        .bind_value_to(self, '_odometry_angular_weight')

            ui.button('Reset', on_click=self.reset) \
                .tooltip('Reset the position to the GNSS measurement or zero position if GNSS is not available or ignored.')
