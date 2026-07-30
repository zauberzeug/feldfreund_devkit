"""The orchestrator drives whatever the navigation hands out, one segment at a time."""
from collections.abc import AsyncIterator

import pytest
import rosys
from rosys.geometry import Point, Pose
from rosys.testing import assert_pose, forward

from feldfreund_devkit.navigation import (
    CannotStop,
    DriveSegment,
    Navigation,
    Orchestrator,
    StaticNavigation,
)


class TwoLegNavigation(StaticNavigation):
    """Two metres forward in two legs, flowing through the joint and resting at the end."""

    def __init__(self) -> None:
        super().__init__(name='Two Legs')

    def generate_path(self) -> list[DriveSegment]:
        return [
            DriveSegment.from_poses(Pose(), Pose(x=1.0), stop_at_end=False),
            DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), stop_at_end=True),
        ]


class EmptyNavigation(StaticNavigation):

    def __init__(self) -> None:
        super().__init__(name='Empty')

    def generate_path(self) -> list[DriveSegment]:
        return []


class RefusingNavigation(Navigation):

    def __init__(self) -> None:
        super().__init__(name='Refusing')

    async def segments(self) -> AsyncIterator[DriveSegment]:
        raise RuntimeError('no route from here')
        yield  # pragma: no cover  # NOTE: makes this an async generator despite the raise


async def test_drives_the_whole_route(devkit_system) -> None:
    orchestrator = Orchestrator(TwoLegNavigation(), devkit_system.driver)
    driven: list[DriveSegment] = []
    orchestrator.SEGMENT_COMPLETED.subscribe(driven.append)
    completed: list[bool] = []
    orchestrator.RUN_COMPLETED.subscribe(lambda: completed.append(True))

    devkit_system.automator.start(orchestrator.run())
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert_pose(2, 0, deg=0, position_tolerance=0.1)
    assert len(driven) == 2
    assert completed == [True]
    assert orchestrator.current_segment is None


async def test_reports_the_segment_being_driven(devkit_system) -> None:
    navigation = TwoLegNavigation()
    orchestrator = Orchestrator(navigation, devkit_system.driver)
    seen: list[tuple[float, int]] = []
    orchestrator.SEGMENT_STARTED.subscribe(lambda s: seen.append((s.end.x, len(navigation.path))))

    devkit_system.automator.start(orchestrator.run())
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert seen == [(1.0, 2), (2.0, 1)], 'each segment is announced as it starts, and popped after'
    assert navigation.path == []


async def test_an_empty_route_finishes_without_driving(devkit_system) -> None:
    orchestrator = Orchestrator(EmptyNavigation(), devkit_system.driver)
    completed: list[bool] = []
    orchestrator.RUN_COMPLETED.subscribe(lambda: completed.append(True))

    await orchestrator.run()

    assert_pose(0, 0, deg=0)
    assert completed == [True], 'an empty route is a finished run, not a refusal'


async def test_a_navigation_may_refuse_to_start(devkit_system) -> None:
    """Refusing is an exception, not an empty route -- the two must stay distinguishable."""
    orchestrator = Orchestrator(RefusingNavigation(), devkit_system.driver)

    with pytest.raises(RuntimeError, match='no route from here'):
        await orchestrator.run()

    assert orchestrator.current_segment is None


class OneLegNavigation(StaticNavigation):
    """Two metres straight ahead, in one go."""

    def __init__(self) -> None:
        super().__init__(name='One Leg')

    def generate_path(self) -> list[DriveSegment]:
        return [DriveSegment.from_poses(Pose(), Pose(x=2.0))]


TOOL_OFFSET = 0.1


async def _until(condition) -> None:
    while not condition():
        await rosys.sleep(0.1)


async def test_a_stop_holds_the_robot_and_then_resumes(devkit_system) -> None:
    """The robot comes to rest with the tool on the target, waits, then drives on to the end."""
    orchestrator = Orchestrator(OneLegNavigation(), devkit_system.driver)
    at_rest: list[float] = []

    async def work() -> None:
        await _until(lambda: devkit_system.driver.pose.x > 0.2)
        async with orchestrator.path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            at_rest.append(devkit_system.driver.pose.x)
            await rosys.sleep(1.0)
            at_rest.append(devkit_system.driver.pose.x)

    devkit_system.automator.start(rosys.automation.parallelize(orchestrator.run(), work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(1.0 - TOOL_OFFSET, abs=0.05), 'the tool, not the origin, lands on the target'
    assert at_rest[1] == pytest.approx(at_rest[0], abs=0.005), 'the robot stays put while the tool works'
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_is_refused_when_nothing_is_driving(devkit_system) -> None:
    orchestrator = Orchestrator(OneLegNavigation(), devkit_system.driver)

    with pytest.raises(CannotStop):
        async with orchestrator.path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            pass


async def test_a_stop_behind_the_robot_is_refused(devkit_system) -> None:
    """Refused rather than reversed: the caller carries on and the robot keeps driving."""
    orchestrator = Orchestrator(OneLegNavigation(), devkit_system.driver)
    refused: list[bool] = []

    async def work() -> None:
        await _until(lambda: devkit_system.driver.pose.x > 1.0)
        try:
            async with orchestrator.path_driver.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                refused.append(False)
        except CannotStop:
            refused.append(True)

    devkit_system.automator.start(rosys.automation.parallelize(orchestrator.run(), work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused == [True]
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_failed_actuation_still_lets_the_robot_go(devkit_system) -> None:
    """The release is in a ``finally``, so a raising tool can never strand the robot stopped."""
    orchestrator = Orchestrator(OneLegNavigation(), devkit_system.driver)

    async def work() -> None:
        await _until(lambda: devkit_system.driver.pose.x > 0.2)
        with pytest.raises(RuntimeError):
            async with orchestrator.path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
                raise RuntimeError('actuation failed')

    devkit_system.automator.start(rosys.automation.parallelize(orchestrator.run(), work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_second_stop_at_the_same_time_is_unsupported(devkit_system) -> None:
    """One stop at a time; a second holder would silently take over the single stop slot."""
    orchestrator = Orchestrator(OneLegNavigation(), devkit_system.driver)
    crashed: list[bool] = []

    async def work() -> None:
        await _until(lambda: devkit_system.driver.pose.x > 0.2)
        async with orchestrator.path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            with pytest.raises(AssertionError):
                async with orchestrator.path_driver.stop_over(Point(x=1.5, y=0.0), TOOL_OFFSET):
                    pass
            crashed.append(True)

    devkit_system.automator.start(rosys.automation.parallelize(orchestrator.run(), work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert crashed == [True]
