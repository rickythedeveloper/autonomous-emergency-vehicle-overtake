from __future__ import annotations
from typing import List, Callable
from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod

import numpy as np

from ...algorithms.continuous.utils.Arc import make_arc
from ...algorithms.continuous.utils.Pose import Pose
from ...algorithms.continuous.utils.heading import clean_heading
from ...utils.Vector2 import Vector2

class VehicleType(Enum):
	civilian = 0
	emergency = 1

class LateralDirection(Enum):
	left = 0
	right = 1

@dataclass
class Control:
	speed: float
	turn_direction: LateralDirection
	turn_radius: float

	@staticmethod
	def zero() -> Control:
		return Control(0, LateralDirection.left, np.Inf)

@dataclass
class ObservedVehicle:
	vehicle_type: VehicleType
	relative_position: Vector2
	relative_velocity: Vector2
	relative_heading: float # between -pi and pi
	contains: Callable[[Vector2], bool] # copy of the contains function of the observed vehicle
	pose_at_time: Callable[[float], Pose | None]

@dataclass
class FuturePose:
	pose: Pose
	time: float

class ContinuousVehicle(ABC):
	_vehicle_type: VehicleType
	observed_vehicles: List[ObservedVehicle]
	position_is_obstacle: Callable[[Vector2], bool] # returns whether the given relative position is in an obstacle
	control: Control = Control.zero()
	future_poses: List[FuturePose]
	_road_heading: float

	def __init__(self, vehicle_type: VehicleType):
		self._vehicle_type = vehicle_type
		self.future_poses = []
		self.observed_vehicles = []
		self._road_heading = 0.0

	@property
	def road_heading(self): return self._road_heading

	@road_heading.setter
	def road_heading(self, value): self._road_heading = clean_heading(value)

	@property
	def vehicle_type(self): return self._vehicle_type

	@property
	@abstractmethod
	def collision_test_points_local_frame(self) -> List[Vector2]:
		"""
		This property contains all the points (in this vehicle's frame) that need to be tested for collision.
		e.g. 100 points around the edges of the vehicle.
		"""
		raise NotImplementedError

	@collision_test_points_local_frame.setter
	@abstractmethod
	def collision_test_points_local_frame(self, value: List[Vector2]):
		raise NotImplementedError

	def position_will_collide(self, relative_position: Vector2, relative_heading: float, time: float) -> bool:
		"""
		Returns whether the given position and heading (in the vehicle frame)
		would collide with another vehicle or an obstacle.

		:param relative_position:  the proposed position in the vehicle frame
		:param relative_heading: relative heading at the proposed position
		:param time: time at which we evaluate the collision
		"""
		for p in self.collision_test_points_local_frame:
			test_point = relative_position + p.rotated_clockwise(relative_heading)

			# check for obstacle
			if self.position_is_obstacle(test_point): return True

			# check for other vehicles
			for v in self.observed_vehicles:
				other_vehicle_pose_at_t = v.pose_at_time(time)
				# if the observed vehicle has no plan registered for time t, then ignore
				if other_vehicle_pose_at_t is None: continue
				# compute relative position of the test point w.r.t. the current position of the observed vehicle
				vehicle_current_to_test_point = (test_point - v.relative_position).rotated_clockwise(-v.relative_heading)
				# compute relative position of the test point w.r.t. the future position (at time t) of the observed vehicle
				vehicle_at_t_to_test_point = (vehicle_current_to_test_point - other_vehicle_pose_at_t.position).rotated_clockwise(-other_vehicle_pose_at_t.heading)
				if v.contains(vehicle_at_t_to_test_point): return True
		return False

	def pose_at_time(self, time: float) -> Pose | None:
		"""Returns its position at a given time in the future. Returns None if the plan does not cover the time given."""
		for index, future_pose in enumerate(self.future_poses):
			if future_pose.time < time: continue
			prev_future_pose = FuturePose(Pose.zero(), 0) if index == 0 else self.future_poses[index - 1]
			arc = make_arc(prev_future_pose.pose, future_pose.pose.position)
			time_proportion = (time - prev_future_pose.time) / (future_pose.time - prev_future_pose.time)
			position = arc.point_on_arc(time_proportion)
			heading = prev_future_pose.pose.heading + time_proportion + (future_pose.pose.heading - prev_future_pose.pose.heading)
			return Pose(position, heading)
		return None

	@abstractmethod
	def contains(self, position: Vector2) -> bool:
		"""
		This function should return whether the given position (in this vehicle's frame) lies within this vehicle.
		"""
		raise NotImplementedError

	# The following two methods will be used instead of compute_velocity when we incorporate the car dynamics.
	@abstractmethod
	def update_control(self) -> None:
		"""This function should update the control at next time step"""
		raise NotImplementedError

	@abstractmethod
	def roll_forward(self, dt: float) -> None:
		"""This function updates any internal vehicle data after the simulator rolls forward by dt seconds"""
		raise NotImplementedError

