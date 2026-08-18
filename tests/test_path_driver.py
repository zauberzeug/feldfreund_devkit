"""Speed-cap composition, and holding the robot at a stop while a tool works.

The stop tests need a robot that is actually driving, so they run a real navigation.
"""
import math
from collections.abc import AsyncGenerator
from types import SimpleNamespace
from typing import NoReturn

import pytest
import rosys
from navigation_helpers import (
    TOOL_OFFSET,
    AheadOfTheRobotNavigation,
    OneLegNavigation,
    RowTurnRowNavigation,
    navigation_run,
    until,
)
from rosys.geometry import Point, Pose
from rosys.testing import assert_pose, forward

from feldfreund_devkit import WorkContext, never
from feldfreund_devkit.navigation import (
    CannotStop,
    DriveSegment,
    Navigation,
    PathDriver,
)

MINIMUM_DRIVE_DISTANCE = 0.005
"""What the configured drive parameters call the shortest worthwhile drive."""


def _path_driver(configured: float = 0.13, *, pose: Pose | None = None) -> PathDriver:
    driver = SimpleNamespace(pose=pose or Pose(),
                             parameters=SimpleNamespace(linear_speed_limit=configured,
                                                        minimum_drive_distance=MINIMUM_DRIVE_DISTANCE))
    return PathDriver(driver)  # type: ignore[arg-type]


def _segment(speed_limit: float | None = None) -> DriveSegment:
    return DriveSegment.from_poses(Pose(), Pose(x=1.0), speed_limit=speed_limit)


@pytest.mark.parametrize(('offset', 'expected'), [
    (0.0, True),
    (MINIMUM_DRIVE_DISTANCE / 2, True),
    (MINIMUM_DRIVE_DISTANCE * 2, False),
    (-MINIMUM_DRIVE_DISTANCE * 2, False),
])
def test_a_target_at_the_tool_needs_no_driving(offset: float, expected: bool) -> None:
    """Unlike a stop, this asks whether the tool is already there -- a target just behind it is not."""
    assert _path_driver().is_reached(Point(x=TOOL_OFFSET + offset, y=0.0), TOOL_OFFSET) is expected


def test_reaching_is_measured_along_the_robots_heading() -> None:
    path_driver = _path_driver(pose=Pose(x=1.0, y=1.0, yaw=math.pi / 2))

    assert path_driver.is_reached(Point(x=1.0, y=1.0 + TOOL_OFFSET), TOOL_OFFSET)
    assert not path_driver.is_reached(Point(x=1.0 + TOOL_OFFSET, y=1.0), TOOL_OFFSET)


@pytest.mark.parametrize(('segment_limit', 'expected'), [(0.05, 0.05), (0.0, 0.0), (0.3, 0.13)])
def test_the_robot_is_never_driven_faster_than_it_is_configured_for(segment_limit: float,
                                                                    expected: float) -> None:
    assert _path_driver(configured=0.13)._effective_speed_limit(_segment(segment_limit)) == expected


def test_a_scoped_cap_applies_only_inside_its_scope() -> None:
    path_driver = _path_driver()
    segment = _segment(0.13)

    with path_driver.limit_speed_to(0.04):
        assert path_driver._effective_speed_limit(segment) == 0.04

    assert path_driver._effective_speed_limit(segment) == 0.13


def test_the_slowest_of_everything_asked_for_wins() -> None:
    path_driver = _path_driver()

    with path_driver.limit_speed_to(0.08), path_driver.limit_speed_to(0.02), path_driver.limit_speed_to(0.5):
        assert path_driver._effective_speed_limit(_segment(0.06)) == 0.02


async def test_a_stop_holds_the_robot_and_then_resumes(devkit_system) -> None:
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())
    at_rest: list[float] = []

    async def work() -> None:
        await until(lambda: devkit_system.driver.pose.x > 0.2)
        async with path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            at_rest.append(devkit_system.driver.pose.x)
            await rosys.sleep(1.0)
            at_rest.append(devkit_system.driver.pose.x)

    devkit_system.automator.start(rosys.automation.parallelize(run, work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(1.0 - TOOL_OFFSET, abs=0.05), 'the tool, not the origin, lands on the target'
    assert at_rest[1] == pytest.approx(at_rest[0], abs=0.005), 'the robot stays put while the tool works'
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_asked_for_before_the_drive_is_honoured_when_it_starts(devkit_system) -> None:
    """A tool need not wait for the first segment: the stop is weighed once there is one to weigh it against."""
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())
    at_rest: list[float] = []

    async def work() -> None:
        async with path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            at_rest.append(devkit_system.driver.pose.x)

    devkit_system.automator.start(rosys.automation.parallelize(run, work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(1.0 - TOOL_OFFSET, abs=0.05)
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_target_already_under_the_tool_is_stopped_at_without_driving(devkit_system) -> None:
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())
    at_rest: list[float] = []

    async def work() -> None:
        async with path_driver.stop_over(Point(x=TOOL_OFFSET, y=0.0), TOOL_OFFSET):
            at_rest.append(devkit_system.driver.pose.x)

    # work first: parallelize steps in order, and one driven tick already carries the tool past a target this close
    devkit_system.automator.start(rosys.automation.parallelize(work(), run))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(0.0, abs=0.001), 'the tool is already there, so the robot does not drive'
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_behind_the_robot_is_refused(devkit_system) -> None:
    """Refused rather than reversed: the caller carries on and the robot keeps driving."""
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())
    refused: list[bool] = []

    async def work() -> None:
        await until(lambda: devkit_system.driver.pose.x > 1.0)
        try:
            async with path_driver.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                refused.append(False)
        except CannotStop:
            refused.append(True)

    devkit_system.automator.start(rosys.automation.parallelize(run, work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused == [True]
    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_failed_actuation_still_lets_the_robot_go(devkit_system) -> None:
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())

    async def work() -> None:
        await until(lambda: devkit_system.driver.pose.x > 0.2)
        with pytest.raises(RuntimeError):
            async with path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
                raise RuntimeError('actuation failed')

    devkit_system.automator.start(rosys.automation.parallelize(run, work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert_pose(2, 0, deg=0, position_tolerance=0.1)


async def test_a_second_stop_at_the_same_time_is_unsupported(devkit_system) -> None:
    """A second holder would silently take over the single stop slot."""
    path_driver, run = navigation_run(devkit_system, OneLegNavigation())
    crashed: list[bool] = []

    async def work() -> None:
        await until(lambda: devkit_system.driver.pose.x > 0.2)
        async with path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            with pytest.raises(AssertionError):
                async with path_driver.stop_over(Point(x=1.5, y=0.0), TOOL_OFFSET):
                    pass
            crashed.append(True)

    devkit_system.automator.start(rosys.automation.parallelize(run, work()))
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert crashed == [True]


async def test_a_stop_on_a_later_segment_is_honoured_when_it_starts(devkit_system) -> None:
    """A tool should not have to know where the segments end; it asks for a stop and waits."""
    at_rest: list[float] = []

    async def work(ctx: WorkContext) -> NoReturn:
        try:
            async with ctx.motion.stop_over(Point(x=1.5, y=0.0), TOOL_OFFSET):
                at_rest.append(ctx.pose.pose.x)
        except CannotStop:
            pass
        await never()

    _, run = navigation_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest, 'the stop was asked for on the first segment and lies on the second'
    assert at_rest[0] == pytest.approx(1.5 - TOOL_OFFSET, abs=0.005), \
        'reduced against the segment that passes the target, not an extrapolation of the current one'
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_far_off_the_segment_is_refused(devkit_system) -> None:
    """Only the segment being driven is known, so a distant target cannot be promised a stop."""
    refused: list[str] = []

    async def work(ctx: WorkContext) -> NoReturn:
        try:
            async with ctx.motion.stop_over(Point(x=3.5, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = navigation_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'off the segment' in refused[0]
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_behind_the_robot_is_refused_off_the_segment_too(devkit_system) -> None:
    refused: list[str] = []

    async def work(ctx: WorkContext) -> NoReturn:
        await until(lambda: devkit_system.driver.pose.x > 1.2)
        try:
            async with ctx.motion.stop_over(Point(x=0.9, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = navigation_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'behind the robot' in refused[0]
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_behind_the_segment_is_refused(devkit_system) -> None:
    """A target the navigation never reaches back to is refused, rather than waited on forever."""
    refused: list[str] = []

    async def work(ctx: WorkContext) -> NoReturn:
        try:
            async with ctx.motion.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = navigation_run(devkit_system, AheadOfTheRobotNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'behind the segment' in refused[0], \
        'the target is ahead of the robot, but behind the segment it is driving onto'


class PlanningPauseNavigation(Navigation):
    """Two workable segments, with the second one planned only once it is asked for."""

    PLANNING_TIME: float = 1.0

    async def segments(self, speed_limit: float) -> AsyncGenerator[DriveSegment, None]:
        yield DriveSegment.from_poses(Pose(), Pose(x=1.0), use_implement=True,
                                      stop_at_end=False, speed_limit=speed_limit)
        await rosys.sleep(self.PLANNING_TIME)
        yield DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), use_implement=True, speed_limit=speed_limit)


async def test_a_stop_survives_the_pause_between_two_segments(devkit_system) -> None:
    """A tool that asks between segments must not lose its target: the navigation drives right over it."""
    completed: list[DriveSegment] = []
    at_rest: list[float] = []
    refused: list[str] = []

    async def work(ctx: WorkContext) -> NoReturn:
        await until(lambda: completed)
        try:
            async with ctx.motion.stop_over(Point(x=1.5, y=0.0), TOOL_OFFSET):
                at_rest.append(ctx.pose.pose.x)
        except CannotStop as e:
            refused.append(str(e))
        await never()

    path_driver, run = navigation_run(devkit_system, PlanningPauseNavigation(), work=work)
    path_driver.SEGMENT_COMPLETED.subscribe(completed.append)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert not refused, f'the target lies on the segment driven next, yet the stop was refused: {refused}'
    assert at_rest[0] == pytest.approx(1.5 - TOOL_OFFSET, abs=0.05)
    assert_pose(2, 0, deg=0, position_tolerance=0.1)
