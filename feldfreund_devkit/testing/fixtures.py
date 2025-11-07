from collections.abc import AsyncGenerator, Generator

import pytest
import rosys
from feldfreund.system import System
from rosys.geometry import GeoPoint, GeoReference
from rosys.hardware import GnssSimulation, ImuSimulation
from rosys.testing import forward, helpers

from feldfreund_devkit.hardware.tracks import TracksSimulation

GEO_REFERENCE = GeoReference(GeoPoint.from_degrees(lat=51.98333489813455, lon=7.434242465994318))
ROBOT_GEO_START_POSITION = GEO_REFERENCE.origin


@pytest.fixture
async def system(rosys_integration, request) -> AsyncGenerator[System, None]:
    s = System(getattr(request, 'param', 'u6'))
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    GeoReference.update_current(GEO_REFERENCE)
    helpers.odometer = s.robot_locator
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(3)
    assert s.gnss.is_connected, 'device should be created'
    assert s.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
async def system_with_acceleration(rosys_integration) -> AsyncGenerator[System, None]:
    s = System('u4', use_acceleration=True)
    assert isinstance(s.feldfreund.wheels, TracksSimulation)
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    GeoReference.update_current(GEO_REFERENCE)
    helpers.odometer = s.robot_locator
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(3)
    assert s.gnss.is_connected, 'device should be created'
    assert s.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
def gnss(system: System) -> GnssSimulation:
    assert isinstance(system.gnss, GnssSimulation)
    return system.gnss


@pytest.fixture
def imu(system: System) -> ImuSimulation:
    assert isinstance(system.feldfreund.imu, ImuSimulation)
    return system.feldfreund.imu


@pytest.fixture
def driving(system: System) -> Generator[System, None, None]:
    """Drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < 10.0:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    system.automator.start(automation())
    yield system


@pytest.fixture
def gnss_driving(system: System, *, drive_distance: float = 10.0) -> Generator[System, None, None]:
    """Use GNSS to drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < drive_distance:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    system.automation_watcher.robot_locator_watch_active = True
    system.automator.start(automation())
    yield system
