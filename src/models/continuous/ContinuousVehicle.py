from __future__ import annotations
from typing import List, Callable
from enum import Enum
from dataclasses import dataclass
from abc import ABC, abstractmethod
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

@dataclass
class ObservedVehicle:
	vehicle_type: VehicleType
	relative_position: Vector2
	relative_velocity: Vector2
	relative_heading: float # between -pi and pi
	contains: Callable[[Vector2], bool] # copy of the contains function of the observed vehicle

class ContinuousVehicle(ABC):
	_vehicle_type: VehicleType
	observed_vehicles: List[ObservedVehicle] = []
	position_is_obstacle: Callable[[Vector2], bool] # returns whether the given relative position is in an obstacle

	def __init__(self, vehicle_type: VehicleType):
		self._vehicle_type = vehicle_type

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

	def position_will_collide(self, relative_position: Vector2, relative_heading: float) -> bool:
		"""
		Returns whether the given position and heading (in the vehicle frame)
		would collide with another vehicle or an obstacle.

		:param relative_position:  the proposed position in the vehicle frame
		:param relative_heading: relative heading at the proposed position
		"""
		for p in self.collision_test_points_local_frame:
			test_point = relative_position + p.rotated_clockwise(-relative_heading)

			# check for obstacle
			if self.position_is_obstacle(test_point): return True

			# check for other vehicles
			for v in self.observed_vehicles:
				rel_pos_this_frame = test_point - v.relative_position
				rel_pos_other_frame = rel_pos_this_frame.rotated_clockwise(v.relative_heading)
				if v.contains(rel_pos_other_frame): return True
		return False

	@abstractmethod
	def contains(self, position: Vector2) -> bool:
		"""
		This function should return whether the given position (in this vehicle's frame) lies within this vehicle.
		"""
		raise NotImplementedError

	# The following two methods will be used instead of compute_velocity when we incorporate the car dynamics.
	@abstractmethod
	def compute_control(self) -> Control:
		"""This function should compute the speed at next time step"""
		raise NotImplementedError

