"""The orchestrator drives whatever the navigation hands out, one segment at a time."""
from collections.abc import AsyncIterator

import pytest
from rosys.geometry import Pose
from rosys.testing import assert_pose, forward

from feldfreund_devkit.navigation import DriveSegment, Navigation, Orchestrator, StaticNavigation


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
