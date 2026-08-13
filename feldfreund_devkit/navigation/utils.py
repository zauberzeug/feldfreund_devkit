import logging
import math
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
from rosys.geometry import GeoReference, Point, Pose, Spline
from rosys.hardware import Gnss
from rosys.helpers import angle

from .drive_segment import DriveSegment

log = logging.getLogger('feldfreund.navigation')


def is_reference_valid(gnss: Gnss | None, *, max_distance: float = 5000.0) -> bool:
    # TODO: remove?
    if gnss is None:
        return True
    if GeoReference.current is None:
        return False
    if gnss.last_measurement is None:
        return False
    if gnss.last_measurement.gps_quality == 0:
        return False
    return gnss.last_measurement.point.distance(GeoReference.current.origin) <= max_distance


def sub_spline(spline: Spline, t_min: float, t_max: float) -> Spline:
    """Creates a new spline from a sub-segment of the given spline"""
    # TODO: move to rosys.geometry.spline
    def split_cubic(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> tuple[tuple[Point, Point, Point, Point], tuple[Point, Point, Point, Point]]:
        """Split a cubic Bezier at t, returns left and right as (start, c1, c2, end)"""
        q0 = p0.interpolate(p1, t)
        q1 = p1.interpolate(p2, t)
        q2 = p2.interpolate(p3, t)
        r0 = q0.interpolate(q1, t)
        r1 = q1.interpolate(q2, t)
        s0 = r0.interpolate(r1, t)
        return (p0, q0, r0, s0), (s0, r1, q2, p3)

    p0, p1, p2, p3 = spline.start, spline.control1, spline.control2, spline.end
    _, (q0, q1, q2, q3) = split_cubic(p0, p1, p2, p3, t_min)
    s = (t_max - t_min) / (1 - t_min) if t_min != 1 else 0.0
    (r0, r1, r2, r3), _ = split_cubic(q0, q1, q2, q3, s)
    return Spline(start=r0, control1=r1, control2=r2, end=r3)


def generate_three_point_turn(end_pose_current_row: Pose,
                              start_pose_next_row: Pose, *,
                              speed_limit: float,
                              radius: float = 1.5,
                              same_row_threshold: float = 0.01) -> list[DriveSegment]:
    """Generates a three-point turn between two poses

    :param end_pose_current_row: the pose of the end of the current row
    :param start_pose_next_row: the pose of the start of the next row
    :param speed_limit: the fastest the legs may be driven
    :param radius: the radius of the turn
    :param same_row_threshold: the threshold distance between the end of the current row and the start of the next row to consider them to be on the same row
    :return: a list of drive segments to perform the turn
    """
    direction_to_start = end_pose_current_row.relative_direction(start_pose_next_row)
    if end_pose_current_row.distance(start_pose_next_row) < same_row_threshold:
        direction_to_start = np.deg2rad(90)
    first_turn_pose = end_pose_current_row.transform_pose(Pose(x=radius,
                                                               y=radius * np.sign(direction_to_start),
                                                               yaw=direction_to_start))
    back_up_pose = start_pose_next_row.transform_pose(Pose(x=-radius,
                                                           y=radius * np.sign(direction_to_start),
                                                           yaw=-direction_to_start))
    backward = first_turn_pose.relative_pose(back_up_pose).x < 0
    return [
        DriveSegment.from_poses(end_pose_current_row, first_turn_pose,
                                stop_at_end=backward, speed_limit=speed_limit),
        DriveSegment.from_poses(first_turn_pose, back_up_pose, backward=backward,
                                stop_at_end=backward, speed_limit=speed_limit),
        DriveSegment.from_poses(back_up_pose, start_pose_next_row, speed_limit=speed_limit),
    ]


def skip_completed_segments(start_pose: Pose,
                            path_segments: list[DriveSegment], *,
                            max_distance: float = 1.0,
                            max_angle: float = np.deg2rad(45),
                            completed_threshold: float = 0.99) -> list[DriveSegment]:
    """Return the tail of ``path_segments`` starting at the segment the robot can pick up next.

    A segment is a candidate if it is not yet (almost) completed, the robot's heading
    aligns with the spline tangent at the closest point within ``max_angle`` (flipped by
    π for backward segments, since the robot faces opposite to its direction of travel),
    and the cross-track distance to the spline is within ``max_distance``. Returns an
    empty list if no segment qualifies.
    """
    log.debug('skip_completed_segments: start=%s, %d segments, max_distance=%.2fm, max_angle=%.1f°',
              start_pose, len(path_segments), max_distance, np.rad2deg(max_angle))
    for i, segment in enumerate(path_segments):
        # search slightly beyond [0, 1] so a robot just before/after the segment still maps cleanly
        t = segment.spline.closest_point(start_pose.x, start_pose.y, t_min=-0.1, t_max=1.1)
        if t > completed_threshold:
            log.debug('  segment %d rejected: completed (t=%.3f > %.2f)', i, t, completed_threshold)
            continue
        expected_yaw = segment.spline.yaw(t) + (np.pi if segment.backward else 0.0)
        heading_offset = angle(start_pose.yaw, expected_yaw)
        if abs(heading_offset) > max_angle:
            log.debug('  segment %d rejected: heading offset %.1f° (max %.1f°)',
                      i, np.rad2deg(heading_offset), np.rad2deg(max_angle))
            continue
        cross_track_distance = start_pose.distance(segment.spline.pose(t))
        if cross_track_distance > max_distance:
            log.debug('  segment %d rejected: cross-track distance %.2fm (max %.2fm)',
                      i, cross_track_distance, max_distance)
            continue
        log.debug('  segment %d accepted (t=%.3f, heading offset=%.1f°, cross-track=%.2fm); returning %d segments',
                  i, t, np.rad2deg(heading_offset), cross_track_distance, len(path_segments) - i)
        return path_segments[i:]
    log.debug('skip_completed_segments: no segment matched from %s among %d candidates',
              start_pose, len(path_segments))
    return []


def pose_with_tool_at(spline: Spline, target: Point, tool_offset_x: float, *,
                      t_min: float = -0.2, t_max: float = 1.2) -> Pose:
    """The pose on ``spline`` from which a tool ``tool_offset_x`` ahead sits on ``target``."""

    t, _ = _solve_tool_t(spline, target, tool_offset_x, t_min, t_max)
    return spline.pose(t)


class Reach(Enum):
    """Where a spline can bring the tool, relative to a target."""

    ON = auto()
    BEHIND = auto()
    """Already past it at the spline's start, and a route only goes forward."""
    BEYOND = auto()
    """Not on this spline, but a later part of the route may still contain it."""


@dataclass(frozen=True)
class ToolReach:
    """Whether a spline brings the tool onto a target, and where along it."""

    where: Reach
    t: float
    """The spline parameter, only meaningful when ``where`` is ``ON``."""


def tool_reach(spline: Spline, target: Point, tool_offset_x: float, *, tolerance: float = 0.0) -> ToolReach:
    """Where along ``spline`` a tool ``tool_offset_x`` ahead of the robot sits on ``target``.

    :param tolerance: how far behind the spline's start the solution may fall and still count as
        reached at ``t = 0``
    """
    t, inside = _solve_tool_t(spline, target, tool_offset_x, 0.0, 1.0)
    if inside:
        return ToolReach(Reach.ON, t)
    if t > 0.0:
        return ToolReach(Reach.BEYOND, t)
    if _forward_distance(spline, target, 0.0) < tool_offset_x - tolerance:
        return ToolReach(Reach.BEHIND, t)
    return ToolReach(Reach.ON, 0.0)


def _solve_tool_t(spline: Spline, target: Point, tool_offset_x: float,
                  t_min: float, t_max: float, iterations: int = 25) -> tuple[float, bool]:
    """Solve ``spline.pose(t).relative_point(target).x == tool_offset_x`` by bisection.

    : return: the parameter clamped to ``[t_min, t_max]``, and whether the solution was inside it
    """
    # NOTE: The forward distance decreases monotonically along the spline, so a bisection converges.
    if _forward_distance(spline, target, t_min) < tool_offset_x:
        return t_min, False
    if _forward_distance(spline, target, t_max) > tool_offset_x:
        return t_max, False
    low, high = t_min, t_max
    for _ in range(iterations):
        middle = (low + high) / 2
        if _forward_distance(spline, target, middle) > tool_offset_x:
            low = middle
        else:
            high = middle
    return (low + high) / 2, True


def _forward_distance(spline: Spline, target: Point, t: float) -> float:
    """How far ahead of ``spline.pose(t)`` the target lies, in that pose's own frame."""
    gx, gy = spline.gx(t), spline.gy(t)
    length = math.hypot(gx, gy)
    if length == 0.0:
        return 0.0
    return ((target.x - spline.x(t)) * gx + (target.y - spline.y(t)) * gy) / length
