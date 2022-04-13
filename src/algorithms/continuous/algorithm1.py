from __future__ import annotations
from typing import List, Callable, Tuple
from enum import Enum
import numpy as np
from random import uniform
from .utils.Arc import Arc, make_arc
from .utils.Pose import Pose
from .utils.heading import clean_heading
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, FuturePose, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2
from .PDFs.NewPDFs import angle_probability_from_pdf

GaussianParameter = Tuple[float, float]
def compute_overall_gaussian_parameter(parameters: List[GaussianParameter]) -> GaussianParameter:
	sum_mu_over_sigma_squared = 0
	sum_one_over_sigma_squared = 0
	for p in parameters:
		mu_i = clean_heading(p[0], -np.pi) # enforce mu_i to be between -pi and pi
		sigma_i = p[1]
		sum_mu_over_sigma_squared += mu_i / (sigma_i ** 2)
		sum_one_over_sigma_squared += 1 / (sigma_i ** 2)
	mu = sum_mu_over_sigma_squared / sum_one_over_sigma_squared
	sigma = np.sqrt(1 / sum_one_over_sigma_squared)
	return mu, sigma

def gaussian_function_generator(parameter: GaussianParameter, normalisation_constant: float) -> Callable[[float], float]:
	def gaussian_function(angle: float) -> float:
		return normalised_angle_gaussian(parameter, angle) * normalisation_constant
	return gaussian_function

def normalised_angle_gaussian(param: GaussianParameter, x: float):
	mu, sigma = param
	x_minus_mu = clean_heading(x - mu, -np.pi) # (x - mu) between -pi and pi
	return np.exp(-0.5 * (x_minus_mu ** 2) / (sigma ** 2))

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

class VehicleStuckError(Exception):
	"""Vehicle cannot plan the next path"""
	pass

class Mode(Enum):
	SIMPLE_CONE = 0
	WITH_ROAD_DIRECTION = 1
	WITH_ROAD_AND_VEHICLES = 2
	HARSH_PDF = 3

NUM_POSES_IN_PLAN = 3
DISTANCE_BETWEEN_POSES = 5
MAX_ARRIVING_ANGLE_DISCREPANCY = np.pi / 10
ARC_SPLIT_LENGTH = 0.3
REMOVE_POSE_TIME = 1
CONE_ANGLE = np.pi / 6
RUN_MODE = Mode.HARSH_PDF
ROAD_HEADING_SIGMA = np.pi / 24

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

	def weight_density_generator(self, position: Vector2, heading: float, time: float) -> Callable[[float], float]:
		if RUN_MODE == Mode.SIMPLE_CONE:
			return lambda angle: 1
		elif RUN_MODE == Mode.WITH_ROAD_DIRECTION:
			return gaussian_function_generator((self.road_heading - heading, ROAD_HEADING_SIGMA), 1)
		elif RUN_MODE == Mode.WITH_ROAD_AND_VEHICLES:
			gaussian_parameters: List[GaussianParameter] = [
				(self.road_heading - heading, ROAD_HEADING_SIGMA)  # road heading
			]

			# avoid other vehicles
			for v in self.observed_vehicles:
				pose = v.pose_at_time(time)
				if pose is None: continue

				v_future_position = v.relative_position + pose.position.rotated_clockwise(v.relative_heading)
				v_future_position_from_pos = (v_future_position - position).rotated_clockwise(-heading)

				mu_opposite = Vector2.zero().heading_to(v_future_position_from_pos)
				mu_v = clean_heading(mu_opposite + np.pi, -np.pi)

				sigma_v = v_future_position_from_pos.length * np.pi / 2  # pi / 2 when distance is 1m
				gaussian_parameters.append((mu_v, sigma_v))

			g_param = compute_overall_gaussian_parameter(gaussian_parameters)

			if g_param[0] < -CONE_ANGLE / 2 or g_param[0] > CONE_ANGLE / 2:
				left_value = normalised_angle_gaussian(g_param, -CONE_ANGLE / 2)
				right_value = normalised_angle_gaussian(g_param, CONE_ANGLE / 2)
				normalisation_constant = 1 / max(left_value, right_value)
			else:
				normalisation_constant = 1

			return gaussian_function_generator(g_param, normalisation_constant)
		elif RUN_MODE == Mode.HARSH_PDF:
			goal_heading_relative = self.road_heading - heading
			car_heading = -goal_heading_relative

			# compute traffic
			traffic: List[List[float]] = []
			for v in self.observed_vehicles:
				pose = v.pose_at_time(time)
				if pose is None: continue
				v_future_position = v.relative_position + pose.position.rotated_clockwise(v.relative_heading)
				v_future_position_from_pos = (v_future_position - position).rotated_clockwise(-heading)
				traffic.append([v_future_position_from_pos.x, v_future_position_from_pos.y])

			def factory(traffic_factory, car_heading_factory) -> Callable[[float], float]:
				def weight_density(angle: float) -> float:
					return angle_probability_from_pdf(
						traffic_factory,
						[0.0, 0.0],
						car_heading_factory * 180 / np.pi,
						angle * 180 / np.pi
					)
				return weight_density

			return factory(traffic, car_heading)
		else:
			raise NotImplementedError

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
			final_time = 0 if len(self.future_poses) == 0 else self.future_poses[-1].time
			weight_density_function = self.weight_density_generator(final_pose.position, final_pose.heading, final_time)
			max_weight = 1
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
						raise VehicleStuckError
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

	# TODO remove these two functions to make civilian cars do PDF
	def roll_forward(self, dt: float):
		return

	def update_control(self):
		self.control = Control(self.speed, LateralDirection.left, 1000000)
		return

EMERGENCY_SPEED = 2
class ContinuousEmergencyVehicle(Algorithm2Vehicle):
	def __init__(self):
		super().__init__(VehicleType.emergency, EMERGENCY_SPEED)