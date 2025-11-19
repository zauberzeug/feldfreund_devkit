import numpy as np
from rosys.geometry import GeoReference, Point, Pose, Spline
from rosys.hardware import Gnss

from .drive_segment import DriveSegment


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
                              radius: float = 1.5,
                              same_row_threshold: float = 0.01) -> list[DriveSegment]:
    """Generates a three-point turn between two poses

    :param end_pose_current_row: the pose of the end of the current row
    :param start_pose_next_row: the pose of the start of the next row
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
        DriveSegment.from_poses(end_pose_current_row, first_turn_pose, stop_at_end=backward),
        DriveSegment.from_poses(first_turn_pose, back_up_pose, backward=backward, stop_at_end=backward),
        DriveSegment.from_poses(back_up_pose, start_pose_next_row),
    ]
