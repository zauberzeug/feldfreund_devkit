"""A run drives whatever the navigation hands out, one segment at a time."""
from collections.abc import AsyncGenerator
from typing import NoReturn

import pytest
from conftest import DRIVE_SPEED
from navigation_helpers import TOOL_OFFSET, RowTurnRowNavigation, navigation_run
from rosys.geometry import Point, Pose
from rosys.testing import assert_pose, forward

from feldfreund_devkit import WorkContext, WorkFunction, never
from feldfreund_devkit.implement import ImplementException
from feldfreund_devkit.navigation import (
    CannotStop,
    DriveSegment,
    Navigation,
    NavigationRefused,
    PathDriver,
    StaticNavigation,
    drive_and_work,
)


class TwoLegNavigation(StaticNavigation):

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        return [
            DriveSegment.from_poses(Pose(), Pose(x=1.0), stop_at_end=False, speed_limit=speed_limit),
            DriveSegment.from_poses(Pose(x=1.0), Pose(x=2.0), stop_at_end=True, speed_limit=speed_limit),
        ]


class EmptyNavigation(StaticNavigation):

    def generate_path(self, speed_limit: float) -> list[DriveSegment]:
        return []


class RefusingNavigation(Navigation):

    async def segments(self, speed_limit: float) -> AsyncGenerator[DriveSegment, None]:
        raise NavigationRefused('no way from here')
        yield  # pragma: no cover  # NOTE: makes this an async generator despite the raise


async def test_drives_the_whole_navigation(devkit_system) -> None:
    path_driver, run = navigation_run(devkit_system, TwoLegNavigation())
    driven: list[DriveSegment] = []
    path_driver.SEGMENT_COMPLETED.subscribe(driven.append)

    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert_pose(2, 0, deg=0, position_tolerance=0.1)
    assert len(driven) == 2


async def test_reports_the_path_as_it_shrinks(devkit_system) -> None:
    navigation = TwoLegNavigation()
    path_driver, run = navigation_run(devkit_system, navigation)
    started: list[float] = []
    path_driver.SEGMENT_STARTED.subscribe(lambda segment: started.append(segment.end.x))
    remaining: list[int] = []
    navigation.PATH_CHANGED.subscribe(lambda path: remaining.append(len(path)))

    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert started == [1.0, 2.0]
    assert remaining == [2, 1, 0], 'a segment leaves the path once it has been driven'


async def test_an_empty_navigation_finishes_without_driving(devkit_system) -> None:
    _, run = navigation_run(devkit_system, EmptyNavigation())

    await run  # an empty navigation is a finished run, not a refusal

    assert_pose(0, 0, deg=0)


async def test_a_navigation_may_refuse_to_start(devkit_system) -> None:
    """Refusing is an exception, not an empty navigation -- the two must stay distinguishable."""
    _, run = navigation_run(devkit_system, RefusingNavigation())

    with pytest.raises(NavigationRefused, match='no way from here'):
        await run


async def test_work_spans_a_stretch_and_never_a_turn(devkit_system) -> None:
    navigation = RowTurnRowNavigation()
    working: list[str] = []

    async def work(ctx: WorkContext) -> NoReturn:
        working.append('start')
        try:
            await never()
        finally:
            working.append(f'end at x={ctx.locator.pose.x:.0f}')

    _, run = navigation_run(devkit_system, navigation, work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert working == ['start', 'end at x=2', 'start', 'end at x=4'], \
        'one stretch per run of workable segments, not one per segment'
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_every_tool_works_every_stretch(devkit_system) -> None:
    working: list[str] = []

    def loop_named(name: str) -> WorkFunction:
        async def work(ctx: WorkContext) -> NoReturn:
            working.append(name)
            await never()
        return work

    run = drive_and_work(RowTurnRowNavigation(), PathDriver(devkit_system.driver), devkit_system.robot_locator,
                         speed_limit=DRIVE_SPEED, work=[loop_named('a'), loop_named('b')])
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert sorted(working) == ['a', 'a', 'b', 'b'], 'both tools, on both stretches'


async def test_tools_take_turns_stopping_the_robot(devkit_system) -> None:
    at_rest: dict[str, float] = {}
    run = drive_and_work(RowTurnRowNavigation(), PathDriver(devkit_system.driver), devkit_system.robot_locator,
                         speed_limit=DRIVE_SPEED, work=[_stopping_at(at_rest, 'a', 0.5, TOOL_OFFSET),
                                                        _stopping_at(at_rest, 'b', 1.0, TOOL_OFFSET)])
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest['a'] == pytest.approx(0.5 - TOOL_OFFSET, abs=0.05)
    assert at_rest['b'] == pytest.approx(1.0 - TOOL_OFFSET, abs=0.05)


async def test_a_tool_waiting_for_its_turn_skips_a_stop_the_robot_has_passed(devkit_system) -> None:
    """Turns go by request order: the second tool's stop lies before the first one's, so it is skipped."""
    at_rest: dict[str, float] = {}
    run = drive_and_work(RowTurnRowNavigation(), PathDriver(devkit_system.driver), devkit_system.robot_locator,
                         speed_limit=DRIVE_SPEED, work=[_stopping_at(at_rest, 'a', 0.5, -0.3),
                                                        _stopping_at(at_rest, 'b', 1.0, 0.4)])
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest['a'] == pytest.approx(0.8, abs=0.05)
    assert 'b' not in at_rest


async def test_a_work_loop_that_returns_is_an_error(devkit_system) -> None:
    """Returning early would silently halt the drive, so it is reported instead."""
    async def work(ctx: WorkContext) -> None:
        return

    _, run = navigation_run(devkit_system, RowTurnRowNavigation(), work=work)  # type: ignore[arg-type]

    with pytest.raises(ImplementException, match='must run until the stretch ends'):
        await run


async def test_work_stops_the_robot_where_the_tool_needs_it(devkit_system) -> None:
    navigation = RowTurnRowNavigation()
    at_rest: list[float] = []

    async def work(ctx: WorkContext) -> NoReturn:
        try:
            async with ctx.motion.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                at_rest.append(ctx.locator.pose.x)
        except CannotStop:
            pass  # NOTE: the second stretch starts past this target, as a real loop must tolerate
        await never()

    _, run = navigation_run(devkit_system, navigation, work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(0.5 - TOOL_OFFSET, abs=0.05)
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


def _stopping_at(at_rest: dict[str, float], name: str, x: float, tool_offset_x: float) -> WorkFunction:
    """A work loop that stops once with its tool on ``x`` and records where the robot came to rest.

    :param at_rest: where each tool's robot came to rest, filled in by name
    """
    async def work(ctx: WorkContext) -> NoReturn:
        try:
            async with ctx.motion.stop_over(Point(x=x, y=0.0), tool_offset_x):
                at_rest[name] = ctx.locator.pose.x
        except CannotStop:
            pass
        await never()
    return work
