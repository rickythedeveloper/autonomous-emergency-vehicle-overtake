from __future__ import annotations
from typing import List, Callable
import numpy as np
from random import uniform
from .utils.Arc import Arc, make_arc
from .utils.Pose import Pose
from .utils.heading import clean_heading
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, FuturePose, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2

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

def pick_angle(weight_density_function: Callable[[float], float], max_weight: float, min_angle: float, max_angle: float):
	while True:
		angle = uniform(min_angle, max_angle)
		value = uniform(0, max_weight)
		if value < weight_density_function(angle): return angle

NUM_POSES_IN_PLAN = 5
DISTANCE_BETWEEN_POSES = 5
MAX_ARRIVING_ANGLE_DISCREPANCY = np.pi / 10
ARC_SPLIT_LENGTH = 0.3
REMOVE_POSE_TIME = 1
CONE_ANGLE = np.pi / 6

class Algorithm2Vehicle(ContinuousVehicle):
	speed: float
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self, vehicle_type: VehicleType, speed: float):
		super().__init__(vehicle_type)
		self.speed = speed
		self.collision_test_points_local_frame = test_points_on_box(self._width, self._length, 0.1)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def weight_density_generator(self, position: Vector2, heading: float) -> Callable[[float], float]:
		def weight_density(angle: float): return 1
		return weight_density

	def max_weight_generator(self, position: Vector2) -> float:
		return 1

	def arc_will_collide(self, arc: Arc, start_time: float):
		"""Check whether the vehicle will collide when running on the arc"""
		num_arcs = int(arc.length / ARC_SPLIT_LENGTH)
		if num_arcs == 0: num_arcs = 1
		little_arc_length = arc.length / num_arcs
		little_arc_duration = little_arc_length / self.speed
		for index, little_arc in enumerate(arc.split(num_arcs)):
			little_arc_end_time = start_time + (index + 1) * little_arc_duration
			if self.position_will_collide(little_arc.end_position, little_arc.end_heading, little_arc_end_time):
				return True
		return False

	def clean_future_poses(self):
		"""
		Clears the future poses if the current arc will lead to a collision
		or if there is a large discrepancy in the arriving angle at the next pose.
		Also removes the next pose if the vehicle is close to it.
		"""
		if len(self.future_poses) == 0: return
		next_pose = self.future_poses[0].pose
		current_arc = make_arc(Pose.zero(), next_pose.position)

		if self.arc_will_collide(current_arc, 0):
			self.future_poses.clear()
			print('deleted plan due to possible collision')
		else:
			arriving_heading_diff = clean_heading(next_pose.heading - current_arc.end_heading, min_value=-np.pi)
			heading_discrepancy = np.abs(arriving_heading_diff)
			if heading_discrepancy > MAX_ARRIVING_ANGLE_DISCREPANCY:
				self.future_poses.clear()
				print("deleted plan due to large discrepancy")
			elif next_pose.position.length < self.speed * REMOVE_POSE_TIME:
				del self.future_poses[0]
				print("reached a pose")

	def add_poses(self):
		"""Keeps adding poses until we have NUM_POSES_IN_PLAN many poses in the plan"""
		back_track_count = 0
		no_plan_collision_count = 0
		while len(self.future_poses) < NUM_POSES_IN_PLAN:
			final_pose = Pose.zero() if len(self.future_poses) == 0 else self.future_poses[-1].pose
			weight_density_function = self.weight_density_generator(final_pose.position, final_pose.heading)
			max_weight = self.max_weight_generator(final_pose.position)
			angle_picked = pick_angle(weight_density_function, max_weight, -CONE_ANGLE / 2, CONE_ANGLE / 2)
			next_position_last_pose_frame = Vector2(np.sin(angle_picked), np.cos(angle_picked)) * DISTANCE_BETWEEN_POSES
			next_position = final_pose.position_relative_to_world(next_position_last_pose_frame)
			new_arc = make_arc(final_pose, next_position)

			# check the path does not collide
			new_arc_start_time = 0.0 if len(self.future_poses) == 0 else self.future_poses[-1].time
			will_collide = self.arc_will_collide(new_arc, new_arc_start_time)
			if will_collide:
				if back_track_count > 20: # TODO constant
					self.future_poses.clear()
					back_track_count = 0
					no_plan_collision_count = 0
					print("too many iterations. clearing plans")
					continue

				if len(self.future_poses) > 0:
					del self.future_poses[-1]
					back_track_count += 1
					print('backtracked')
				else:
					no_plan_collision_count += 1
					print('has no plan but a proposed path will collide', no_plan_collision_count)
					if no_plan_collision_count > 20:
						raise Exception('has no plan, and a proposed path will collide')
			else:
				arc_time = new_arc.length / self.speed
				end_time = arc_time if len(self.future_poses) == 0 else self.future_poses[-1].time + arc_time
				self.future_poses.append(FuturePose(Pose(next_position, new_arc.end_heading), end_time))
				no_plan_collision_count = 0
				print("added a pose")

	def rrt(self):
		self.clean_future_poses()
		self.add_poses()

	def roll_forward(self, dt: float):
		"""Update the content of the future poses based on the current control"""
		assert self.control is not None
		delta_distance = self.speed * dt
		delta_heading = delta_distance / self.control.turn_radius * (1 if self.control.turn_direction == LateralDirection.right else -1)
		half_delta_heading = delta_heading / 2  # heading at halfway point

		velocity = Vector2(np.sin(half_delta_heading), np.cos(half_delta_heading)) * self.speed
		delta_position = velocity * dt

		future_poses: List[FuturePose] = []
		for index, p in enumerate(self.future_poses):
			rel_pos_previous_frame = p.pose.position - delta_position
			rel_pos_next_frame = rel_pos_previous_frame.rotated_clockwise(-delta_heading)
			future_poses.append(FuturePose(Pose(rel_pos_next_frame, p.pose.heading - delta_heading), p.time - dt))
		self.future_poses = future_poses

	def update_control(self):
		assert not self.position_will_collide(Vector2.zero(), 0, 0), 'The vehicle has collisded'

		self.rrt()

		next_pose = self.future_poses[0].pose

		arc = make_arc(Pose.zero(), next_pose.position)
		direction = LateralDirection.right if arc.circle.center.x > 0 else LateralDirection.left

		self.control = Control(self.speed, direction, arc.circle.radius)

CIVILIAN_SPEED = 1
class ContinuousCivilianVehicle(Algorithm2Vehicle):
	def __init__(self):
		super().__init__(VehicleType.civilian, CIVILIAN_SPEED)
		for n in range(NUM_POSES_IN_PLAN):
			y = (n + 1) * DISTANCE_BETWEEN_POSES
			self.future_poses.append(FuturePose(
				Pose(Vector2(0, y), 0),
				y / CIVILIAN_SPEED
			))

EMERGENCY_SPEED = 2
class ContinuousEmergencyVehicle(Algorithm2Vehicle):
	def __init__(self):
		super().__init__(VehicleType.emergency, EMERGENCY_SPEED)