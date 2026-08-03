"""The orchestrator drives whatever the navigation hands out, one segment at a time."""
from collections.abc import AsyncIterator

import pytest
from rosys.geometry import Point, Pose
from rosys.testing import assert_pose, forward
from route_helpers import TOOL_OFFSET, NoDetection, RowTurnRowNavigation

from feldfreund_devkit.implement import ImplementException
from feldfreund_devkit.navigation import (
    CannotStop,
    DriveSegment,
    Navigation,
    Orchestrator,
    StaticNavigation,
    never,
    no_work,
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
    orchestrator = Orchestrator(TwoLegNavigation(), devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=no_work)
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


async def test_reports_the_route_as_it_shrinks(devkit_system) -> None:
    navigation = TwoLegNavigation()
    orchestrator = Orchestrator(navigation, devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=no_work)
    started: list[float] = []
    orchestrator.SEGMENT_STARTED.subscribe(lambda segment: started.append(segment.end.x))
    remaining: list[int] = []
    navigation.PATH_CHANGED.subscribe(lambda path: remaining.append(len(path)))

    devkit_system.automator.start(orchestrator.run())
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert started == [1.0, 2.0]
    assert remaining == [2, 1, 0], 'a segment leaves the route once it has been driven'


async def test_an_empty_route_finishes_without_driving(devkit_system) -> None:
    orchestrator = Orchestrator(EmptyNavigation(), devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=no_work)
    completed: list[bool] = []
    orchestrator.RUN_COMPLETED.subscribe(lambda: completed.append(True))

    await orchestrator.run()

    assert_pose(0, 0, deg=0)
    assert completed == [True], 'an empty route is a finished run, not a refusal'


async def test_a_navigation_may_refuse_to_start(devkit_system) -> None:
    """Refusing is an exception, not an empty route -- the two must stay distinguishable."""
    orchestrator = Orchestrator(RefusingNavigation(), devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=no_work)

    with pytest.raises(RuntimeError, match='no route from here'):
        await orchestrator.run()


async def test_work_spans_a_stretch_and_never_a_turn(devkit_system) -> None:
    """Work covers consecutive workable segments as one stretch, and is cancelled before the turn."""
    navigation = RowTurnRowNavigation()
    working: list[str] = []

    async def work(ctx) -> None:
        working.append('start')
        try:
            await never()
        finally:
            working.append(f'end at x={ctx.pose.pose.x:.0f}')

    orchestrator = Orchestrator(navigation, devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=work)
    devkit_system.automator.start(orchestrator.run())
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert working == ['start', 'end at x=2', 'start', 'end at x=4'], \
        'one stretch per run of workable segments, not one per segment'
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_work_loop_that_returns_is_an_error(devkit_system) -> None:
    """Returning early would silently halt the drive, so it is reported instead."""
    async def work(ctx) -> None:
        return

    orchestrator = Orchestrator(RowTurnRowNavigation(), devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=work)

    with pytest.raises(ImplementException, match='must run until the stretch ends'):
        await orchestrator.run()


async def test_work_stops_the_robot_where_the_tool_needs_it(devkit_system) -> None:
    """The whole point: a tool can hold the robot at a target from inside its own loop."""
    navigation = RowTurnRowNavigation()
    at_rest: list[float] = []

    async def work(ctx) -> None:
        try:
            async with ctx.motion.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                at_rest.append(ctx.pose.pose.x)
        except CannotStop:
            pass  # NOTE: the second stretch starts past this target, as a real loop must tolerate
        await never()

    orchestrator = Orchestrator(navigation, devkit_system.driver, devkit_system.robot_locator,
                                detection=NoDetection(), work=work)
    devkit_system.automator.start(orchestrator.run())
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest[0] == pytest.approx(0.5 - TOOL_OFFSET, abs=0.05)
    assert_pose(4, 0, deg=0, position_tolerance=0.1)
