from __future__ import annotations
from typing import List, Callable
import numpy as np
from random import uniform
from .utils.Arc import Arc, make_arc
from .utils.Pose import Pose
from .utils.heading import clean_heading
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2

def weight_density_generator(position: Vector2, heading: float) -> Callable[[float], float]:
	angle_range = np.pi / 6

	def weight_density(angle: float):
		angle_modulo = angle % (2 * np.pi)
		if angle_modulo < angle_range / 2 or angle_modulo > 2 * np.pi - angle_range / 2: return 1
		return 0

	return weight_density

def max_weight_generator(position: Vector2) -> float: return 1

def test_points_on_box(width: float, length: float, spacing: float = 0.1) -> List[Vector2]:
	half_width, half_length = width / 2, length / 2
	test_points = []
	# front and back
	for y in [-half_length, half_length]:
		for x in np.arange(-half_width, half_length, spacing):
			test_points.append(Vector2(x, y))

	# both sides
	for x in [-half_width, half_width]:
		for y in np.arange(-half_length, half_length, spacing):
			test_points.append(Vector2(x, y))

	return test_points

def pick_angle(weight_density_function: Callable[[float], float], max_weight: float):
	while True:
		angle = uniform(0, 2 * np.pi)
		value = uniform(0, max_weight)
		if value < weight_density_function(angle): return angle

class ContinuousCivilianVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self):
		super().__init__(VehicleType.civilian)
		self.collision_test_points_local_frame = test_points_on_box(self._width, self._length, 0.1)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def update_control(self):
		self.control = Control(0, LateralDirection.left, 50)

	def roll_forward(self, dt: float) -> None:
		return

NUM_POSES_IN_PLAN = 10
DISTANCE_BETWEEN_POSES = 5
MAX_ARRIVING_ANGLE_DISCREPANCY = np.pi / 10
ARC_SPLIT_LENGTH = 0.3

class ContinuousEmergencyVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []
	future_poses: List[Pose] = []

	def __init__(self):
		super().__init__(VehicleType.emergency)
		self.collision_test_points_local_frame = test_points_on_box(self._width, self._length, 0.1)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def arc_will_collide(self, arc: Arc):
		"""Check whether the vehicle will collide when running on the arc"""
		for little_arc in arc.split(int(arc.length / ARC_SPLIT_LENGTH)):
			if self.position_will_collide(little_arc.end_position, little_arc.end_heading):
				return True
		return False

	def clean_future_poses(self):
		"""
		Clears the future poses if the current arc will lead to a collision
		or if there is a large discrepancy in the arriving angle at the next pose.
		Also removes the next pose if the vehicle is close to it.
		"""
		if len(self.future_poses) == 0: return
		next_pose = self.future_poses[0]
		current_arc = make_arc(Pose.zero(), next_pose.position)

		if self.arc_will_collide(current_arc):
			self.future_poses.clear()
			print('deleted plan due to possible collision')
		else:
			arriving_heading_diff = clean_heading(next_pose.heading - current_arc.end_heading, min_value=-np.pi)
			heading_discrepancy = np.abs(arriving_heading_diff)
			if heading_discrepancy > MAX_ARRIVING_ANGLE_DISCREPANCY:
				self.future_poses.clear()
				print("deleted plan due to large discrepancy")
			elif next_pose.position.length < self.control.speed * 1:
				del self.future_poses[0]
				print("reached a pose")

	def add_poses(self):
		"""Keeps adding poses until we have NUM_POSES_IN_PLAN many poses in the plan"""
		while len(self.future_poses) < NUM_POSES_IN_PLAN:
			final_pose = Pose.zero() if len(self.future_poses) == 0 else self.future_poses[-1]
			weight_density_function = weight_density_generator(final_pose.position, final_pose.heading)
			max_weight = max_weight_generator(final_pose.position)
			angle_picked = pick_angle(weight_density_function, max_weight)
			next_position_last_pose_frame = Vector2(np.sin(angle_picked), np.cos(angle_picked)) * DISTANCE_BETWEEN_POSES
			next_position = final_pose.position_relative_to_world(next_position_last_pose_frame)
			new_arc = make_arc(final_pose, next_position)

			# check the path does not collide
			will_collide = self.arc_will_collide(new_arc)
			if will_collide:
				# TODO better backtracking
				if len(self.future_poses) > 0:
					del self.future_poses[-1]
					print('backtracked')
			else:
				self.future_poses.append(Pose(next_position, new_arc.end_heading))
				print("added a pose")

	def rrt(self):
		self.clean_future_poses()
		self.add_poses()

	def roll_forward(self, dt: float):
		"""Update the content of the future poses based on the current control"""
		assert self.control is not None
		delta_distance = self.control.speed * dt
		delta_heading = delta_distance / self.control.turn_radius * (1 if self.control.turn_direction == LateralDirection.right else -1)
		half_delta_heading = delta_heading / 2  # heading at halfway point

		velocity = Vector2(np.sin(half_delta_heading), np.cos(half_delta_heading)) * self.control.speed
		delta_position = velocity * dt

		future_poses: List[Pose] = []
		for index, p in enumerate(self.future_poses):
			rel_pos_previous_frame = p.position - delta_position
			rel_pos_next_frame = rel_pos_previous_frame.rotated_clockwise(-delta_heading)
			future_poses.append(Pose(rel_pos_next_frame, p.heading - delta_heading))
		self.future_poses = future_poses

	def update_control(self):
		assert not self.position_will_collide(Vector2.zero(), 0), 'The vehicle has collisded'

		self.rrt()

		next_pose = self.future_poses[0]

		arc = make_arc(Pose.zero(), next_pose.position)
		direction = LateralDirection.right if arc.circle.center.x > 0 else LateralDirection.left

		self.control = Control(1, direction, arc.circle.radius)