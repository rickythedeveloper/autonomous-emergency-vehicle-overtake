from __future__ import annotations
from typing import List, Callable
from dataclasses import dataclass
import numpy as np
from random import uniform
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2


class ContinuousCivilianVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self):
		super().__init__(VehicleType.civilian)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def update_control(self):
		self.control = Control(2, LateralDirection.left, 50)

	def roll_forward(self, dt: float) -> None:
		return


def weight_density_generator(position: Vector2) -> Callable[[float], float]:
	def weight_density(angle: float):
		angle_modulo = angle % (2 * np.pi)
		if angle_modulo < np.pi / 6 or angle_modulo > (2 - 1/6) * np.pi: return 1
		return 0
	return lambda x: weight_density(x)

def max_weight_generator(position: Vector2) -> float:
	return 1


def pick_angle(position: Vector2, get_weight_density_function: Callable[[Vector2], Callable[[float], float]], get_max_weight: Callable[[Vector2], float]):
	while True:
		max_weight = get_max_weight(position)
		weight_density_function = get_weight_density_function(position)
		angle = uniform(0, 2 * np.pi)
		value = uniform(0, max_weight)
		if value < weight_density_function(angle): return angle

@dataclass
class Pose:
	position: Vector2
	_heading: float

	def __init__(self, position: Vector2, heading: float):
		self.position = position
		self.heading = heading

	@property
	def heading(self): return self._heading

	@heading.setter
	def heading(self, value: float): self._heading = value % (2 * np.pi)

	@staticmethod
	def zero() -> Pose: return Pose(Vector2(0, 0), 0)

@dataclass
class Circle:
	center: Vector2
	radius: float

@dataclass
class Arc:
	pose1: Pose
	pose2: Pose
	circle: Circle

def make_arc(start_pose: Pose, goal_position: Vector2) -> Arc:
	relative_position = goal_position - start_pose.position
	if relative_position.y == 0:
		phi = (np.pi / 2 if relative_position.x > 0 else -np.pi / 2) - start_pose.heading
	else:
		phi = np.arctan(relative_position.x / relative_position.y) - start_pose.heading

	goal_heading = start_pose.heading + 2 * phi
	goal_pose = Pose(goal_position, goal_heading)
	circle = get_circle(start_pose, goal_pose)
	return Arc(start_pose, goal_pose, circle)

def get_circle(pose1: Pose, pose2: Pose) -> Circle:
	a1 = Vector2.from_heading(pose1.heading).rotated_clockwise(np.pi / 2).slope
	a2 = Vector2.from_heading(pose2.heading).rotated_clockwise(np.pi / 2).slope

	radius1_is_horizontal = np.abs(a1) > 1e5
	radius2_is_horizontal = np.abs(a2) > 1e5

	if not radius1_is_horizontal:
		c1 = pose1.position.y - a1 * pose1.position.x
	if not radius2_is_horizontal:
		c2 = pose2.position.y - a2 * pose2.position.x

	if a1 == a2 or (radius1_is_horizontal and radius2_is_horizontal):
		center = (pose1.position + pose2.position) / 2
	elif radius1_is_horizontal:
		x_center = pose1.position.x
		y_center = a2 * x_center + c2
		center = Vector2(x_center, y_center)
	elif radius2_is_horizontal:
		x_center = pose2.position.x
		y_center = a1 * x_center + c1
		center = Vector2(x_center, y_center)
	else:
		x_center = (c2 - c1) / (a1 - a2)
		y_center = a1 * x_center + c1
		center = Vector2(x_center, y_center)
	radius = (pose1.position - center).length
	radius2 = (pose2.position - center).length
	assert radius2 * 0.99 < radius < radius2 * 1.01
	return Circle(center, radius)

def subtract_heading(h1: float, h2: float) -> float:
	diff = h1 - h2
	while diff < -np.pi: diff += 2 * np.pi
	while diff > np.pi: diff -= 2 * np.pi
	return diff

class ContinuousEmergencyVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []
	future_poses: List[Pose] = []

	def __init__(self):
		super().__init__(VehicleType.emergency)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def rrt(self):
		if len(self.future_poses) > 0:
			next_pose = self.future_poses[0]
			arc = make_arc(Pose.zero(), next_pose.position)
			if np.abs(subtract_heading(next_pose.heading, arc.pose2.heading)) < np.pi / 10:
				# remove next pose if close
				if next_pose.position.length < self.control.speed * 1:
					del self.future_poses[0]
					print("delete next future pose")
			else:
				print(next_pose.heading, arc.pose2.heading)
				# if next pose seems unlikely, remove all future plan
				self.future_poses = []
				print("delete future poses")

		# add future poses
		while len(self.future_poses) < 5:
			tip_point = Pose.zero() if len(self.future_poses) == 0 else self.future_poses[-1]
			angle = pick_angle(tip_point.position, weight_density_generator, max_weight_generator)
			distance = 10 # TODO
			next_position = tip_point.position + Vector2(np.sin(angle), np.cos(angle)) * distance
			arc = make_arc(tip_point, next_position)
			next_heading = arc.pose2.heading
			if not self.position_will_collide(next_position, next_heading):
				self.future_poses.append(Pose(next_position, next_heading))
				print("appended", self.future_poses[-1], next_position, next_heading)

	def roll_forward(self, dt: float):
		assert self.control is not None
		delta_distance = self.control.speed * dt
		delta_heading = delta_distance / self.control.turn_radius * (1 if self.control.turn_direction == LateralDirection.right else -1)
		half_delta_heading = delta_heading / 2  # heading at halfway point

		velocity = Vector2(np.sin(half_delta_heading), np.cos(half_delta_heading)) * self.control.speed
		delta_position = velocity * dt

		future_poses: List[Pose] = []
		for index, p in enumerate(self.future_poses):
			rel_pos_previous_frame = p.position - delta_position
			rel_pos_next_frame = rel_pos_previous_frame.rotated_clockwise(delta_heading)
			future_poses.append(Pose(rel_pos_next_frame, p.heading - delta_heading))
		self.future_poses = future_poses

	def update_control(self):
		self.rrt()

		next_pose = self.future_poses[0]

		arc = make_arc(Pose.zero(), next_pose.position)
		direction = LateralDirection.right if arc.circle.center.x > 0 else LateralDirection.left

		self.control = Control(1, direction, arc.circle.radius)