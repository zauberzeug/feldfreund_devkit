from __future__ import annotations

import gc
import logging
from abc import abstractmethod
from typing import Any

import rosys
from nicegui import Event, ui
from rosys.analysis import track
from rosys.driving import Driver
from rosys.driving.driver import PoseProvider
from rosys.geometry import Point, Pose, PoseStep

from ..implements.implement import Implement
from .drive_segment import DriveSegment
from .utils import sub_spline


class WaypointNavigation(rosys.persistence.Persistable):
    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, *, implement: Implement, driver: Driver, pose_provider: PoseProvider, name: str = 'Waypoint Navigation') -> None:
        super().__init__()
        self.log = logging.getLogger('feldfreund.navigation')
        self.implement = implement
        self.driver = driver
        self.pose_provider = pose_provider
        self.name = name
        self._upcoming_path: list[DriveSegment] = []
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT

        self.PATH_GENERATED = Event[list[DriveSegment]]()
        """a new path has been generated (argument: ``list[DriveSegment]``)"""

        self.SEGMENT_STARTED = Event[DriveSegment]()
        """a waypoint has been reached"""

        self.SEGMENT_COMPLETED = Event[DriveSegment]()
        """a waypoint has been reached"""

        self.PATH_COMPLETED = Event[[]]()
        """the entire path with all its waypoints has been completed"""

    @property
    def path(self) -> list[DriveSegment]:
        return self._upcoming_path

    @property
    def current_segment(self) -> DriveSegment | None:
        if not self._upcoming_path:
            return None
        return self._upcoming_path[0]

    @property
    def has_waypoints(self) -> bool:
        """Returns True as long as there are waypoints to drive to"""
        return self.current_segment is not None

    @track
    async def prepare(self) -> bool:
        """Prepares the navigation for the start of the automation

        Returns true if all preparations were successful, otherwise false."""
        self._upcoming_path = self.generate_path()
        if not self._upcoming_path:
            self.log.error('Path generation failed')
            return False
        self.PATH_GENERATED.emit(self._upcoming_path)
        return True

    @abstractmethod
    def generate_path(self) -> list[DriveSegment]:
        raise NotImplementedError('Subclasses must implement this method')

    @track
    async def start(self) -> None:
        try:
            # TODO
            # if not is_reference_valid(self.gnss):
            #     rosys.notify('GNSS not available or reference too far away', 'warning')
            #     await rosys.sleep(3)
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            if not await self.implement.prepare():
                self.log.error('Implement preparation failed')
                return
            await self.implement.activate()
            rosys.notify('Automation started')
            self.log.debug('Navigation started')

            assert self.current_segment is not None
            self.SEGMENT_STARTED.emit(self.current_segment)
            while self.has_waypoints:
                await self._run()
                await rosys.sleep(0.1)
            rosys.notify('Automation finished', 'positive')
            self.PATH_COMPLETED.emit()
        except Exception as e:
            rosys.notify('Automation failed', 'negative')
            self.log.exception('Navigation failed: %s', e)
        finally:
            await self.implement.finish()
            await self.finish()
            await self.implement.deactivate()
            await self.driver.wheels.stop()

    async def _run(self) -> None:
        if not await self._get_valid_implement_target():
            self.log.debug('No move target found, continuing...')
            await rosys.automation.parallelize(
                self._drive_along_segment(linear_speed_limit=self.linear_speed_limit),
                self._block_until_implement_has_target(),
                return_when_first_completed=True,
            )
        if not self.has_waypoints:
            return
        assert self.current_segment is not None
        # TODO
        # if isinstance(self.implement, WeedingImplement) and self.current_segment.use_implement:
        if self.current_segment.use_implement:
            implement_target = await self._get_valid_implement_target()
            if not implement_target:
                self.log.debug('Implement has no target anymore. Possibly overshot, continuing...')
                return
            if not await self._follow_segment_until(implement_target):
                # TODO
                # assert isinstance(self.detector, rosys.vision.Detector)
                # await self.detector.NEW_DETECTIONS.emitted(5)
                return
            await self.driver.wheels.stop()
            self.implement.has_plants_to_handle()
            await self.implement.start_workflow()
            await self.implement.stop_workflow()

    @track
    async def finish(self) -> None:
        """Executed after the navigation is done"""
        self.log.debug('Navigation finished')
        gc.collect()  # NOTE: auto garbage collection is deactivated to avoid hiccups from Global Interpreter Lock (GIL) so we collect here to reduce memory pressure

    @track
    async def _drive_along_segment(self, *, linear_speed_limit: float = 0.3) -> None:
        """Drive the robot to the next waypoint of the navigation"""
        segment = self.current_segment
        if segment is None:
            return
        stop_at_end = segment.stop_at_end or len(self._upcoming_path) == 1
        with self.driver.parameters.set(linear_speed_limit=linear_speed_limit, can_drive_backwards=segment.backward):
            await self.driver.drive_spline(segment.spline, flip_hook=segment.backward, throttle_at_end=stop_at_end, stop_at_end=stop_at_end)
        self._upcoming_path.pop(0)
        self.SEGMENT_COMPLETED.emit(segment)
        if self.has_waypoints:
            assert self.current_segment is not None
            self.SEGMENT_STARTED.emit(self.current_segment)

    async def _block_until_implement_has_target(self) -> Point:
        while True:
            assert isinstance(self.current_segment, DriveSegment)
            if (target := await self._get_valid_implement_target()):
                return target
            await rosys.sleep(0.1)

    def _remove_segments_behind_robot(self, path_segments: list[DriveSegment]) -> list[DriveSegment]:
        """Create new path (list of segments) starting at the closest segment to the current pose"""
        current_pose = self.pose_provider.pose
        start_index = 0
        for i, segment in enumerate(path_segments):
            t = segment.spline.closest_point(current_pose.x, current_pose.y, t_min=-0.1, t_max=1.1)
            if t > 0.99:
                continue
            start_index = i
            break
        return path_segments[start_index:]

    @track
    async def _follow_segment_until(self, target: Point) -> bool:
        """Drives to a target point along the current spline.

        :param target: The target point to drive to
        """
        current_segment = self.current_segment
        if current_segment is None:
            return False
        current_pose = self.pose_provider.pose
        spline = current_segment.spline
        current_t = spline.closest_point(current_pose.x, current_pose.y)
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        work_x_corrected_pose = self._target_pose_on_current_segment(target)
        distance_to_target = current_pose.distance(work_x_corrected_pose)
        target_t = spline.closest_point(work_x_corrected_pose.x, work_x_corrected_pose.y, t_min=-0.2, t_max=1.2)
        if abs(distance_to_target) < self.driver.parameters.minimum_drive_distance:
            # TODO: quickfix for weeds behind the robot
            self.log.debug('Target close, working with out advancing... (%.6f m)', distance_to_target)
            return True
        if target_t < current_t or target_t > 1.0:
            # TODO: we need a sturdy function to advance a certain distance on a spline, because this method is off by a tiny amount. That's why +0.00003
            # test_weeding.py::test_advance_when_target_behind_robot tests this case. The weed is skipped in this case
            advance_distance = self.driver.parameters.minimum_drive_distance
            while True:
                target_pose = current_pose + PoseStep(linear=advance_distance, angular=0.0, time=0.0)
                target_t = spline.closest_point(target_pose.x, target_pose.y)
                advance_spline = sub_spline(spline, current_t, target_t)
                if advance_spline.estimated_length() > self.driver.parameters.minimum_drive_distance:
                    break
                advance_distance += 0.00001
            self.log.debug('Target behind robot, continue for %.6f meters', advance_distance)
            with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
                await self.driver.drive_spline(advance_spline, throttle_at_end=False, stop_at_end=False)
            return False
        self.log.debug('Driving to %s from target %s', work_x_corrected_pose, target)
        target_spline = sub_spline(spline, current_t, target_t)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
            await self.driver.drive_spline(target_spline)
        return True

    def _target_pose_on_current_segment(self, target: Point) -> Pose:
        assert self.current_segment is not None
        spline = self.current_segment.spline
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        target_pose = spline.pose(target_t)
        # TODO: get self.system.field_friend.WORK_X from implement
        work_x = 0.085
        return target_pose + PoseStep(linear=-work_x, angular=0, time=0)

    async def _get_valid_implement_target(self) -> Point | None:
        if self.current_segment is None or not self.current_segment.use_implement:
            return None
        implement_target = await self.implement.get_target()
        if not implement_target:
            return None
        t = self.current_segment.spline.closest_point(implement_target.x, implement_target.y)
        if t in (0.0, 1.0):
            self.log.debug('Target is on segment end, continuing...')
            return None
        work_x_corrected_pose = self._target_pose_on_current_segment(implement_target)
        distance_to_target = self.pose_provider.pose.distance(work_x_corrected_pose)
        t = self.current_segment.spline.closest_point(work_x_corrected_pose.x, work_x_corrected_pose.y)
        if t in (0.0, 1.0) and abs(distance_to_target) > self.driver.parameters.minimum_drive_distance:
            # TODO: quickfix for weeds behind the robot
            self.log.debug('WorkX corrected target is on segment end, continuing...')
            return None
        return implement_target

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'linear_speed_limit': self.linear_speed_limit,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)

    def settings_ui(self) -> None:
        ui.number('Linear Speed',
                  step=0.01,
                  min=self.driver.parameters.throttle_at_end_min_speed,
                  max=self.driver.parameters.linear_speed_limit,
                  format='%.2f',
                  suffix='m/s',
                  on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'linear_speed_limit') \
            .tooltip(f'Forward speed limit between {self.driver.parameters.throttle_at_end_min_speed} and {self.driver.parameters.linear_speed_limit} m/s (default: {self.LINEAR_SPEED_LIMIT:.2f})')

    def developer_ui(self) -> None:
        pass
