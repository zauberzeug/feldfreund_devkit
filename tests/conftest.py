# pylint: disable=unused-argument
from collections.abc import AsyncGenerator, Generator

import pytest
import rosys
from rosys.automation import Automator
from rosys.driving import Driver, Steerer
from rosys.geometry import GeoPoint, GeoReference, Pose
from rosys.hardware import GnssSimulation, ImuSimulation, WheelsSimulation
from rosys.testing import forward, helpers

from feldfreund_devkit.config import config_from_id, create_drive_parameters
from feldfreund_devkit.hardware.tracks import TracksSimulation
from feldfreund_devkit.implement import ImplementDummy
from feldfreund_devkit.navigation import StraightLineNavigation
from feldfreund_devkit.robot_locator import RobotLocator
from feldfreund_devkit.system import System

GEO_REFERENCE = GeoReference(GeoPoint.from_degrees(lat=51.98333489813455, lon=7.434242465994318))
ROBOT_GEO_START_POSITION = GEO_REFERENCE.origin


class TestSystem(System):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        GeoReference.update_current(GEO_REFERENCE)
        self.robot_locator = RobotLocator(self.feldfreund.wheels,
                                          gnss=self.feldfreund.gnss,
                                          imu=self.feldfreund.imu,
                                          gnss_config=self.config.gnss)
        self.steerer = Steerer(self.feldfreund.wheels, speed_scaling=0.25)
        self.driver = Driver(self.feldfreund.wheels, self.robot_locator, parameters=create_drive_parameters())
        self.automator = Automator(self.steerer, on_interrupt=self.feldfreund.stop, notify=False)
        helpers.odometer = self.odometer
        helpers.driver = self.driver
        helpers.automator = self.automator
        self.current_implement = ImplementDummy()
        self.current_navigation = StraightLineNavigation(implement=self.current_implement,
                                                         driver=self.driver,
                                                         pose_provider=self.robot_locator)
        self.automator.default_automation = self.current_navigation.start

    def set_robot_pose(self, pose: Pose):
        # pylint: disable=protected-access
        assert isinstance(self.feldfreund.wheels, WheelsSimulation)
        self.robot_locator._reset(x=pose.x, y=pose.y, yaw=pose.yaw)
        self.feldfreund.wheels.pose.x = pose.x
        self.feldfreund.wheels.pose.y = pose.y
        self.feldfreund.wheels.pose.yaw = pose.yaw


@pytest.fixture
async def devkit_system(rosys_integration) -> AsyncGenerator[TestSystem, None]:
    config = config_from_id('example')
    s = TestSystem(config)
    await forward(3)
    assert s.feldfreund.gnss is not None
    assert s.feldfreund.gnss.is_connected, 'device should be created'
    assert s.feldfreund.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.feldfreund.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
async def devkit_system_with_acceleration(rosys_integration) -> AsyncGenerator[TestSystem, None]:
    config = config_from_id('example')
    s = TestSystem(config, use_acceleration=True)
    assert isinstance(s.feldfreund.wheels, TracksSimulation)
    await forward(3)
    assert s.feldfreund.gnss is not None
    assert s.feldfreund.gnss.is_connected, 'device should be created'
    assert s.feldfreund.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.feldfreund.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
def gnss(request: pytest.FixtureRequest) -> GnssSimulation:
    s = request.getfixturevalue('devkit_system')
    assert isinstance(s.feldfreund.gnss, GnssSimulation)
    return s.feldfreund.gnss


@pytest.fixture
def imu(request: pytest.FixtureRequest) -> ImuSimulation:
    s = request.getfixturevalue('devkit_system')
    assert isinstance(s.feldfreund.imu, ImuSimulation)
    return s.feldfreund.imu


@pytest.fixture
def driving(request: pytest.FixtureRequest, *, drive_distance: float = 10.0) -> Generator[TestSystem, None, None]:
    """Drive 10 meters in a straight line"""
    s = request.getfixturevalue('devkit_system')

    async def automation():
        while s.driver.prediction.point.x < drive_distance:
            await s.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    s.automator.start(automation())
    yield s


@pytest.fixture
def gnss_driving(request: pytest.FixtureRequest, *, drive_distance: float = 10.0) -> Generator[TestSystem, None, None]:
    """Use GNSS to drive 10 meters in a straight line"""
    s = request.getfixturevalue('devkit_system')

    async def automation():
        while s.driver.prediction.point.x < drive_distance:
            await s.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    s.automator.start(automation())
    yield s
