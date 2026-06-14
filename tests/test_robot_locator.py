import numpy as np
import pytest
import rosys
from rosys.testing import forward


async def test_covariance_stays_positive_semidefinite(devkit_system):
    """Regression test for #348 (cholesky: matrix not positive definite) and
    #372 (invalid value in matmul).

    Both stem from the naive covariance update ``P = (I - K H) P``: while driving
    forward, ``F`` couples position and heading, so ``P`` gains off-diagonal terms
    and the naive update lets it drift into an asymmetric, non-positive-definite
    matrix (even negative variances). That makes ``cholesky(S)`` fail and, once the
    resulting garbage turns into NaN, propagates into the predict step ``F @ P @ F.T``.

    The covariance must stay symmetric, positive semi-definite and finite throughout.
    """
    s = devkit_system

    async def drive_curve():
        for _ in range(400):
            await s.driver.wheels.drive(0.3, 0.3)  # forward + turn couples position and heading
            await rosys.sleep(0.1)
    s.automator.start(drive_curve())
    await forward(until=lambda: s.automator.is_running)

    for _ in range(200):
        await forward(0.2)
        covariance = s.robot_locator._Sxx
        assert np.all(np.isfinite(covariance)), f'covariance must stay finite:\n{covariance}'
        eigenvalues = np.linalg.eigvalsh((covariance + covariance.T) / 2)
        scale = max(float(eigenvalues.max()), 1e-30)
        assert eigenvalues.min() >= -1e-9 * scale, \
            f'covariance must stay positive semi-definite, got eigenvalues {eigenvalues}:\n{covariance}'
        asymmetry = float(np.max(np.abs(covariance - covariance.T)))
        assert asymmetry <= 1e-9 * scale, f'covariance must stay symmetric (asymmetry {asymmetry:.2e}):\n{covariance}'


async def test_covariance_stays_finite_while_driving(devkit_system):
    """Regression test for #372: the predict step ``F @ P @ F.T`` must never see NaN/Inf."""
    s = devkit_system

    async def drive_curve():
        for _ in range(400):
            await s.driver.wheels.drive(0.3, 0.3)
            await rosys.sleep(0.1)
    s.automator.start(drive_curve())
    await forward(until=lambda: s.automator.is_running)
    for _ in range(200):
        await forward(0.2)
        assert np.all(np.isfinite(s.robot_locator._Sxx)), 'covariance turned non-finite'
        assert np.all(np.isfinite(s.robot_locator._x)), 'state turned non-finite'


async def test_dead_reckoning_integrates_odometry(devkit_system):
    """Without GNSS the prediction step alone must move the pose along the driven odometry."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(5.0)
    await s.driver.wheels.drive(0.0, 0.0)
    await forward(0.5)
    assert s.robot_locator.pose.x == pytest.approx(s.feldfreund.wheels.pose.x, abs=0.02)
    assert s.robot_locator.pose.y == pytest.approx(0.0, abs=0.02)
    assert s.robot_locator.pose.x > 0.5  # actually moved


async def test_standstill_freezes_state(devkit_system):
    """At standstill the prediction step is skipped, so pose and covariance must not drift."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(1.0)
    await s.driver.wheels.drive(0.0, 0.0)
    await forward(0.5)
    pose_before = s.robot_locator.pose
    covariance_before = s.robot_locator._Sxx.copy()
    await forward(3.0)
    assert s.robot_locator.pose.x == pytest.approx(pose_before.x, abs=1e-9)
    assert s.robot_locator.pose.y == pytest.approx(pose_before.y, abs=1e-9)
    assert s.robot_locator.pose.yaw == pytest.approx(pose_before.yaw, abs=1e-9)
    np.testing.assert_array_equal(s.robot_locator._Sxx, covariance_before)


async def test_uncertainty_grows_during_dead_reckoning(devkit_system):
    """Driving without GNSS corrections must increase the pose uncertainty."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(0.5)
    uncertainty_before = sum(s.robot_locator.uncertainty)
    await forward(5.0)
    uncertainty_after = sum(s.robot_locator.uncertainty)
    assert uncertainty_after > uncertainty_before


async def test_gnss_corrects_injected_error(devkit_system):
    """A GNSS update must pull an injected pose error back towards the measurement.

    The robot has to be moving: at standstill the prediction step is skipped, the
    covariance keeps shrinking and the filter becomes overconfident, so it would no
    longer correct. Driving keeps the process noise (and thus the Kalman gain) alive.
    """
    s = devkit_system
    await forward(2.0)  # let the filter settle at the origin
    s.robot_locator._x[1, 0] = 1.0  # inject a 1 m error in y
    s.robot_locator._update_frame()
    assert s.robot_locator.pose.y == pytest.approx(1.0, abs=1e-6)
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(5.0)
    await s.driver.wheels.drive(0.0, 0.0)
    assert s.robot_locator.pose.y == pytest.approx(0.0, abs=0.05)


async def test_non_finite_heading_disables_update(devkit_system):
    """If the GNSS heading uncertainty is not finite, the update is skipped entirely
    (the Feldfreund relies on RTK heading), so an injected error must NOT be corrected
    even while driving (which otherwise keeps the Kalman gain alive)."""
    s = devkit_system
    await forward(2.0)
    s.feldfreund.gnss._heading_std_dev = np.inf
    await forward(1.0)  # flush a measurement so the guard is exercised
    s.robot_locator._x[1, 0] = 1.0
    s.robot_locator._update_frame()
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(5.0)
    await s.driver.wheels.drive(0.0, 0.0)
    assert s.robot_locator.pose.y == pytest.approx(1.0, abs=1e-6)  # uncorrected (straight drive keeps y in odometry)


async def test_reset_snaps_pose_to_gnss(devkit_system):
    """Resetting must discard the current state and adopt the GNSS measurement.

    ``reset`` awaits a *new* GNSS measurement, which only arrives while simulated time
    advances, so it is run as a background task alongside ``forward``.
    """
    s = devkit_system
    s.robot_locator._x[:, 0] = [5.0, 5.0, 1.0]  # corrupt the state
    s.robot_locator._update_frame()
    rosys.background_tasks.create(s.robot_locator.reset(), name='reset_locator')
    await forward(3.0)
    assert s.robot_locator.pose.x == pytest.approx(s.feldfreund.wheels.pose.x, abs=0.05)
    assert s.robot_locator.pose.y == pytest.approx(s.feldfreund.wheels.pose.y, abs=0.05)


async def test_pose_at_interpolates_past_pose(devkit_system):
    """``pose_at`` must return the pose estimated at an earlier time, not the live pose."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(1.5)
    now = rosys.time()
    current_x = s.robot_locator.pose.x
    past = s.robot_locator.pose_at(now - 1.0)
    assert past.x < current_x  # the earlier estimate is behind the current one
    assert past.x == pytest.approx(current_x - 0.2 * 1.0, abs=0.05)  # ~0.2 m/s over 1 s
    assert past.y == pytest.approx(0.0, abs=0.02)


async def test_pose_at_clamps_to_current_for_future(devkit_system):
    """A timestamp at or beyond the newest estimate returns the live pose."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(1.0)
    future = s.robot_locator.pose_at(rosys.time() + 10.0)
    assert future.x == pytest.approx(s.robot_locator.pose.x)
    assert future.yaw == pytest.approx(s.robot_locator.pose.yaw)


async def test_pose_history_is_time_windowed(devkit_system):
    """The buffer keeps only roughly ``POSE_HISTORY_DURATION`` seconds of estimates."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(5.0)
    history = s.robot_locator._pose_history
    assert len(history) > 1
    span = history[-1][0] - history[0][0]
    assert span <= s.robot_locator.POSE_HISTORY_DURATION + 0.5


async def test_pose_at_interpolates_yaw_across_turn(devkit_system):
    """Yaw must be interpolated with wrap-around handling while turning."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.0, 0.3)  # turn in place
    await forward(1.5)
    now = rosys.time()
    current_yaw = s.robot_locator.pose.yaw
    past = s.robot_locator.pose_at(now - 1.0)
    assert abs(past.yaw) < abs(current_yaw)  # earlier in the turn ⇒ smaller angle


async def test_reset_clears_pose_history(devkit_system):
    """Resetting discards the buffered estimates so interpolation never crosses the jump."""
    s = devkit_system
    s.robot_locator._ignore_gnss = True
    await s.driver.wheels.drive(0.2, 0.0)
    await forward(1.0)
    assert len(s.robot_locator._pose_history) > 1
    s.robot_locator._reset()
    assert len(s.robot_locator._pose_history) == 1  # only the post-reset estimate remains


def test_combine_odom_imu_slip_reduces_speed(devkit_system):
    """When odometry and IMU disagree on the angular velocity (wheel slip), the
    fused linear velocity must be reduced and the angular velocity blended."""
    locator = devkit_system.robot_locator
    weight = locator._odometry_angular_weight
    # agreement: speed unchanged, angular velocity unchanged
    v, omega = locator._combine_odom_imu(0.5, 0.2, 0.2)
    assert v == pytest.approx(0.5)
    assert omega == pytest.approx(0.2)
    # disagreement: linear speed reduced, angular velocity blended towards the IMU
    v, omega = locator._combine_odom_imu(0.5, 0.2, 1.2)
    assert v < 0.5
    assert omega == pytest.approx(weight * 0.2 + (1 - weight) * 1.2)
