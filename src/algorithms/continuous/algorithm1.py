from __future__ import annotations
from typing import List, Callable
from enum import Enum
import numpy as np

from .errors import VehicleStuckError
from .gaussian import weight_density_generator_road_and_vehicle, weight_density_generator_with_road
from .pick_angle import pick_angle
from .test_points import test_points_on_box
from .utils.Arc import Arc, make_arc
from .utils.Pose import Pose
from .utils.heading import clean_heading
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, FuturePose, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2
from .PDFs.NewPDFs import angle_probability_from_pdf

class Mode(Enum):
	SIMPLE_CONE = 0
	WITH_ROAD_DIRECTION = 1
	WITH_ROAD_AND_VEHICLES = 2
	HARSH_PDF = 3

RUN_MODE = Mode.WITH_ROAD_AND_VEHICLES

NUM_POSES_IN_PLAN = 4
MAX_COLLISION_COUNT_BEFORE_BACKTRACK = 5
MAX_COLLISION_COUNT_BEFORE_TERMINATION = 20
MAX_BACKTRACK_COUNT_FOR_CLEARING = 5
MAX_CLEAR_COUNT = 5
MAX_ARRIVING_ANGLE_DISCREPANCY = np.pi / 10
ARC_SPLIT_LENGTH = 0.1
REMOVE_POSE_TIME = 0.6
MIN_TURNING_RADIUS = 10

class Algorithm1Vehicle(ContinuousVehicle):
	speed: float
	distance_between_poses: float
	cone_angle: float
	wasted_proposal_count: int
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self, vehicle_type: VehicleType, speed: float):
		super().__init__(vehicle_type)
		self.speed = speed
		self.distance_between_poses = speed * 2
		self.cone_angle = np.arccos(1 - 0.5 * (self.distance_between_poses / MIN_TURNING_RADIUS) ** 2)
		self.wasted_proposal_count = 0
		self.collision_test_points_local_frame = test_points_on_box(self._width, self._length, 0.1)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def weight_density_generator(self, position: Vector2, heading: float, time: float) -> Callable[[float], float]:
		if RUN_MODE == Mode.SIMPLE_CONE:
			return lambda angle: 1
		elif RUN_MODE == Mode.WITH_ROAD_DIRECTION:
			return weight_density_generator_with_road(self.road_heading - heading)
		elif RUN_MODE == Mode.WITH_ROAD_AND_VEHICLES:
			traffic_positions: List[Vector2] = []
			vehicle_types: List[VehicleType] = []
			for v in self.observed_vehicles:
				pose = v.pose_at_time(time)
				if pose is None: continue

				v_future_position = v.relative_position + pose.position.rotated_clockwise(v.relative_heading)
				v_future_position_from_pos = (v_future_position - position).rotated_clockwise(-heading)
				traffic_positions.append(v_future_position_from_pos)
				vehicle_types.append(v.vehicle_type)
			return weight_density_generator_road_and_vehicle(traffic_positions, vehicle_types, self.road_heading - heading, self.cone_angle)
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
						angle * 180 / np.pi,
						self.cone_angle * 180 / np.pi
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
		collision_count = 0
		backtrack_count = 0
		clear_count = 0
		while len(self.future_poses) < NUM_POSES_IN_PLAN:
			final_pose = Pose.zero() if len(self.future_poses) == 0 else self.future_poses[-1].pose
			final_time = 0 if len(self.future_poses) == 0 else self.future_poses[-1].time
			weight_density_function = self.weight_density_generator(final_pose.position, final_pose.heading, final_time)
			max_weight = 1
			angle_picked = pick_angle(weight_density_function, max_weight, -self.cone_angle / 2, self.cone_angle / 2)
			next_position_last_pose_frame = Vector2(np.sin(angle_picked), np.cos(angle_picked)) * self.distance_between_poses
			next_position = final_pose.position_relative_to_world(next_position_last_pose_frame)
			new_arc = make_arc(final_pose, next_position)

			# check the path does not collide
			new_arc_start_time = 0.0 if len(self.future_poses) == 0 else self.future_poses[-1].time
			will_collide = self.arc_will_collide(new_arc, new_arc_start_time)
			if will_collide:
				self.wasted_proposal_count += 1
				if collision_count < MAX_COLLISION_COUNT_BEFORE_BACKTRACK:
					collision_count += 1
					# print('collision. no backtrack')
					continue
				if len(self.future_poses) == 0:
					if collision_count < MAX_COLLISION_COUNT_BEFORE_TERMINATION:
						collision_count += 1
						# print('collision, but has no plan, so keep going')
						continue
					print(f'vehicle could not propose a new path with no plan')
					raise VehicleStuckError
				if backtrack_count < MAX_BACKTRACK_COUNT_FOR_CLEARING:
					self.wasted_proposal_count += 1
					del self.future_poses[-1]
					backtrack_count += 1
					collision_count = 0
					print('backtracked')
					continue
				if clear_count < MAX_CLEAR_COUNT:
					self.wasted_proposal_count += len(self.future_poses)
					self.future_poses.clear()
					clear_count += 1
					backtrack_count = 0
					collision_count = 0
					print('cleared')
					continue
				print(f'vehicle cleared plan too many times')
				raise VehicleStuckError
			else:
				arc_time = new_arc.length / self.speed
				end_time = arc_time if len(self.future_poses) == 0 else self.future_poses[-1].time + arc_time
				self.future_poses.append(FuturePose(Pose(next_position, new_arc.end_heading), end_time))
				collision_count = 0
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
		if self.position_will_collide(Vector2.zero(), 0, 0):
			print('vehicle has collided')
			raise VehicleStuckError

		self.rrt()

		next_pose = self.future_poses[0].pose

		arc = make_arc(Pose.zero(), next_pose.position)
		direction = LateralDirection.right if arc.circle.center.x > 0 else LateralDirection.left

		self.control = Control(self.speed, direction, arc.circle.radius)

CIVILIAN_SPEED = 1
class ContinuousCivilianVehicle(Algorithm1Vehicle):
	def __init__(self):
		super().__init__(VehicleType.civilian, CIVILIAN_SPEED)
		# for n in range(NUM_POSES_IN_PLAN):
		# 	y = (n + 1) * self.distance_between_poses
		# 	self.future_poses.append(FuturePose(
		# 		Pose(Vector2(0, y), 0),
		# 		y / CIVILIAN_SPEED
		# 	))

	# uncomment these two functions to make civilian cars go straight
	# def roll_forward(self, dt: float):
	# 	return
	#
	# def update_control(self):
	# 	self.control = Control(self.speed, LateralDirection.left, 1000000)
	# 	return

EMERGENCY_SPEED = 2
class ContinuousEmergencyVehicle(Algorithm1Vehicle):
	def __init__(self):
		super().__init__(VehicleType.emergency, EMERGENCY_SPEED)