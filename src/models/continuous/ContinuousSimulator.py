from typing import List, Callable
from dataclasses import dataclass
import numpy as np
from .ContinuousVehicle import ContinuousVehicle, LateralDirection, ObservedVehicle
from ...utils.Vector2 import Vector2

@dataclass
class VehicleData:
	object: ContinuousVehicle
	position: Vector2
	velocity: Vector2
	_heading: float # between zero and two pi

	@property
	def heading(self): return self._heading

	@heading.setter
	def heading(self, value: float):
		# Make sure the heading is between zero and two pi
		self._heading = value
		while self._heading < 0: self._heading += 2 * np.pi
		while self._heading > 2 * np.pi: self._heading -= 2 * np.pi

	def position_relative_to_world(self, relative_position: Vector2):
		return relative_position.rotated_clockwise(-self.heading) + self.position

	def position_world_to_relative(self, world_position: Vector2):
		return (world_position - self.position).rotated_clockwise(self.heading)

	def velocity_relative_to_world(self, relative_velocity: Vector2):
		return relative_velocity.rotated_clockwise(-self.heading) + self.velocity

	def velocity_world_to_relative(self, world_velocity: Vector2):
		return (world_velocity - self.velocity).rotated_clockwise(self.heading)

class ContinuousSimulator:
	vehicles: List[VehicleData]
	position_is_obstacle: Callable[[Vector2], bool]

	def __init__(self, position_is_obstacle: Callable[[Vector2], bool], vehicles: List[VehicleData] = None):
		self.position_is_obstacle = position_is_obstacle
		self.vehicles = []
		if vehicles is not None:
			for v in vehicles:
				self.add_vehicle(v)

	def add_vehicle(self, vehicle: VehicleData):
		assert vehicle not in self.vehicles
		self.vehicles.append(vehicle)

	def remove_vehicle(self, vehicle: VehicleData):
		assert vehicle in self.vehicles
		delete_index = self.vehicles.index(vehicle)
		del self.vehicles[delete_index]

	def update_observed_data(self):
		"""Updates information held by each vehicle"""
		for v1 in self.vehicles:
			# update observed vehicles
			v1.object.observed_vehicles = []
			for v2 in self.vehicles:
				if v1 is v2: continue
				# TODO limit the communication radius
				relative_position = v1.position_world_to_relative(v2.position)
				relative_velocity = v1.velocity_world_to_relative(v2.velocity)
				relative_heading = v2.heading - v1.heading
				# make sure relative heading is between -pi and pi
				while relative_heading < -np.pi: relative_heading += 2 * np.pi
				while relative_heading > np.pi: relative_heading -= 2 * np.pi

				def v2_contains(position_v1_frame: Vector2) -> bool:
					world_position = v1.position_relative_to_world(position_v1_frame)
					position_v2_frame = v2.position_world_to_relative(world_position)
					return v2.object.contains(position_v2_frame)

				v1.object.observed_vehicles.append(ObservedVehicle(
					v2.object.vehicle_type,
					relative_position,
					relative_velocity,
					relative_heading,
					v2_contains
				))

			# update obstacle info
			v1.object.position_is_obstacle = \
				lambda rel_pos: self.position_is_obstacle(v1.position_relative_to_world(rel_pos))

	def roll_forward(self, dt: float):
		self.update_observed_data()

		# compute velocity and position based on the new observed data
		for vehicle in self.vehicles:
			control = vehicle.object.compute_control()
			delta_distance = control.speed * dt
			delta_heading = delta_distance / control.turn_radius * (1 if control.turn_direction == LateralDirection.right else -1)
			average_heading = vehicle.heading + delta_heading / 2 # heading at halfway point

			# update velocity, position and heading
			vehicle.velocity = Vector2(np.sin(average_heading), np.cos(average_heading)) * control.speed
			vehicle.position += vehicle.velocity * dt
			vehicle.heading += delta_heading