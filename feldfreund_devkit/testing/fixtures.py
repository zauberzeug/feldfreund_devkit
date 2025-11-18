from collections.abc import AsyncGenerator, Generator

import pytest
import rosys
from rosys.automation import Automator
from rosys.driving import Driver, Steerer
from rosys.geometry import GeoPoint, GeoReference
from rosys.hardware import GnssSimulation, ImuSimulation
from rosys.testing import forward, helpers

from feldfreund_devkit.config import create_drive_parameters
from feldfreund_devkit.hardware.tracks import TracksSimulation
from feldfreund_devkit.implement import ImplementDummy
from feldfreund_devkit.navigation import StraightLineNavigation
from feldfreund_devkit.robot_locator import RobotLocator
from feldfreund_devkit.system import System

GEO_REFERENCE = GeoReference(GeoPoint.from_degrees(lat=51.98333489813455, lon=7.434242465994318))
ROBOT_GEO_START_POSITION = GEO_REFERENCE.origin


@pytest.fixture
async def devkit_system(rosys_integration) -> AsyncGenerator[System, None]:
    s = System('example')
    GeoReference.update_current(GEO_REFERENCE)
    s.robot_locator = RobotLocator(s.feldfreund.wheels,
                                   gnss=s.feldfreund.gnss,
                                   imu=s.feldfreund.imu,
                                   gnss_config=s.config.gnss)
    s.steerer = Steerer(s.feldfreund.wheels, speed_scaling=0.25)
    s.driver = Driver(s.feldfreund.wheels, s.robot_locator, parameters=create_drive_parameters())
    s.automator = Automator(s.steerer, on_interrupt=s.feldfreund.stop, notify=False)
    helpers.odometer = s.odometer
    helpers.driver = s.driver
    helpers.automator = s.automator
    s.current_implement = ImplementDummy()
    s.current_navigation = StraightLineNavigation(implement=s.current_implement,
                                                  driver=s.driver,
                                                  pose_provider=s.robot_locator)
    s.automator.default_automation = s.current_navigation.start
    await forward(3)
    assert s.feldfreund.gnss.is_connected, 'device should be created'
    assert s.feldfreund.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.feldfreund.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
async def devkit_system_with_acceleration(rosys_integration) -> AsyncGenerator[System, None]:
    s = System('example', use_acceleration=True)
    assert isinstance(s.feldfreund.wheels, TracksSimulation)
    GeoReference.update_current(GEO_REFERENCE)
    s.robot_locator = RobotLocator(s.feldfreund.wheels,
                                   gnss=s.feldfreund.gnss,
                                   imu=s.feldfreund.imu,
                                   gnss_config=s.config.gnss)
    s.steerer = Steerer(s.feldfreund.wheels, speed_scaling=0.25)
    s.driver = Driver(s.feldfreund.wheels, s.robot_locator, parameters=create_drive_parameters())
    s.automator = Automator(s.steerer, on_interrupt=s.feldfreund.stop, notify=False)
    helpers.odometer = s.odometer
    helpers.driver = s.driver
    helpers.automator = s.automator
    s.current_implement = ImplementDummy()
    s.current_navigation = StraightLineNavigation(implement=s.current_implement,
                                                  driver=s.driver,
                                                  pose_provider=s.robot_locator)
    s.automator.default_automation = s.current_navigation.start
    await forward(3)
    assert s.feldfreund.gnss.is_connected, 'device should be created'
    assert s.feldfreund.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.feldfreund.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
def gnss(devkit_system: System) -> GnssSimulation:
    assert isinstance(devkit_system.feldfreund.gnss, GnssSimulation)
    return devkit_system.feldfreund.gnss


@pytest.fixture
def imu(devkit_system: System) -> ImuSimulation:
    assert isinstance(devkit_system.feldfreund.imu, ImuSimulation)
    return devkit_system.feldfreund.imu


@pytest.fixture
def driving(devkit_system: System) -> Generator[System, None, None]:
    """Drive 10 meters in a straight line"""
    async def automation():
        while devkit_system.driver.prediction.point.x < 10.0:
            await devkit_system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    devkit_system.automator.start(automation())
    yield devkit_system


@pytest.fixture
def gnss_driving(devkit_system: System, *, drive_distance: float = 10.0) -> Generator[System, None, None]:
    """Use GNSS to drive 10 meters in a straight line"""
    async def automation():
        while devkit_system.driver.prediction.point.x < drive_distance:
            await devkit_system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    devkit_system.automation_watcher.robot_locator_watch_active = True
    devkit_system.automator.start(automation())
    yield devkit_system
