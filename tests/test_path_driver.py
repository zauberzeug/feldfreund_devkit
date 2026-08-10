"""Speed-cap composition, and holding the robot at a stop while a tool works.

The stop tests need a robot that is actually driving, so they run a real route -- but what they
pin is ``PathDriver`` behaviour.
"""
import pytest
import rosys
from rosys.geometry import Point, Pose
from rosys.testing import assert_pose, forward
from route_helpers import (
    TOOL_OFFSET,
    AheadOfTheRobotNavigation,
    OneLegNavigation,
    RowTurnRowNavigation,
    route_run,
    until,
)

from feldfreund_devkit import never
from feldfreund_devkit.navigation import (
    CannotStop,
    DriveSegment,
    PathDriver,
)


def _path_driver() -> PathDriver:
    return PathDriver(driver=None)  # type: ignore[arg-type]


def _segment(speed_limit: float | None = None) -> DriveSegment:
    return DriveSegment.from_poses(Pose(), Pose(x=1.0), speed_limit=speed_limit)


@pytest.mark.parametrize(('segment_limit', 'expected'), [(None, 0.13), (0.05, 0.05), (0.0, 0.0), (0.3, 0.13)])
def test_a_segment_can_only_slow_the_robot_down(segment_limit: float | None, expected: float) -> None:
    assert _path_driver().speed_limit(_segment(segment_limit), 0.13) == expected


def test_a_scoped_cap_applies_only_inside_its_scope() -> None:
    path_driver = _path_driver()
    segment = _segment()

    with path_driver.limit(0.04):
        assert path_driver.speed_limit(segment, 0.13) == 0.04

    assert path_driver.speed_limit(segment, 0.13) == 0.13


def test_the_slowest_of_everything_asked_for_wins() -> None:
    """Caps compose: the segment, the user and every scope get a veto, none can speed things up."""
    path_driver = _path_driver()

    with path_driver.limit(0.08), path_driver.limit(0.02), path_driver.limit(0.5):
        assert path_driver.speed_limit(_segment(0.06), 0.13) == 0.02


async def test_a_stop_holds_the_robot_and_then_resumes(devkit_system) -> None:
    """The robot comes to rest with the tool on the target, waits, then drives on to the end."""
    path_driver, run = route_run(devkit_system, OneLegNavigation())
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


async def test_a_stop_is_refused_when_nothing_is_driving(devkit_system) -> None:
    path_driver, _ = route_run(devkit_system, OneLegNavigation())

    with pytest.raises(CannotStop):
        async with path_driver.stop_over(Point(x=1.0, y=0.0), TOOL_OFFSET):
            pass


async def test_a_stop_behind_the_robot_is_refused(devkit_system) -> None:
    """Refused rather than reversed: the caller carries on and the robot keeps driving."""
    path_driver, run = route_run(devkit_system, OneLegNavigation())
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
    """The release is in a ``finally``, so a raising tool can never strand the robot stopped."""
    path_driver, run = route_run(devkit_system, OneLegNavigation())

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
    """One stop at a time; a second holder would silently take over the single stop slot."""
    path_driver, run = route_run(devkit_system, OneLegNavigation())
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
    """A stop stays pending across segment boundaries.

    The driver follows one spline at a time, but a tool should not have to know where the segments
    end -- it asks for a stop and waits, and whichever segment contains it comes to rest there.
    """
    at_rest: list[float] = []

    async def work(ctx) -> None:
        try:
            async with ctx.motion.stop_over(Point(x=1.5, y=0.0), TOOL_OFFSET):
                at_rest.append(ctx.pose.pose.x)
        except CannotStop:
            pass
        await never()

    _, run = route_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert at_rest, 'the stop was asked for on the first segment and lies on the second'
    assert at_rest[0] == pytest.approx(1.5 - TOOL_OFFSET, abs=0.005), \
        'reduced against the segment that passes the target, not an extrapolation of the current one'
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_far_off_the_segment_is_refused(devkit_system) -> None:
    """Only the segment being driven is known, so a distant target cannot be promised a stop.

    Accepting one would be worse than a stall: the reduction clamps to the segment's parameter
    range, so a target metres beyond the end comes back as a pose just past it -- the robot would
    stop somewhere nobody asked for.
    """
    refused: list[str] = []

    async def work(ctx) -> None:
        try:
            async with ctx.motion.stop_over(Point(x=3.5, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = route_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'off the segment' in refused[0]
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_behind_the_robot_is_refused_off_the_segment_too(devkit_system) -> None:
    """A target behind the robot is refused even when it falls off the segment being driven.

    The reduction extrapolates past a segment's start, so such a pose is not "on" the segment and
    would otherwise pass the on-segment check and be waited on forever.
    """
    refused: list[str] = []

    async def work(ctx) -> None:
        await until(lambda: devkit_system.driver.pose.x > 1.2)
        try:
            async with ctx.motion.stop_over(Point(x=0.9, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = route_run(devkit_system, RowTurnRowNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'behind the robot' in refused[0]
    assert_pose(4, 0, deg=0, position_tolerance=0.1)


async def test_a_stop_behind_the_segment_is_refused(devkit_system) -> None:
    """A target the route never reaches back to is refused, rather than waited on forever.

    ``tool_t`` answers ``None`` both for a target beyond a segment's end and for one before its
    start. The first is worth waiting for -- a later segment will contain it -- while the second
    never comes, because a route only goes forward.
    """
    refused: list[str] = []

    async def work(ctx) -> None:
        try:
            async with ctx.motion.stop_over(Point(x=0.5, y=0.0), TOOL_OFFSET):
                pass
        except CannotStop as e:
            refused.append(str(e))
        await never()

    _, run = route_run(devkit_system, AheadOfTheRobotNavigation(), work=work)
    devkit_system.automator.start(run)
    await forward(until=lambda: devkit_system.automator.is_running)
    await forward(until=lambda: devkit_system.automator.is_stopped)

    assert refused and 'behind the segment' in refused[0], \
        'the target is ahead of the robot, but behind the segment it is driving onto'
