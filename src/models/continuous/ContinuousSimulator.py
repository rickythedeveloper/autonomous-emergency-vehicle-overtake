from typing import List, Callable
from .ContinuousVehicle import ContinuousVehicle
from ...utils.Vector2 import Vector2

class ContinuousSimulator:
	vehicles: List[ContinuousVehicle]
	position_is_obstacle: Callable[[Vector2], bool]

	def __init__(self, position_is_obstacle: Callable[[Vector2], bool], vehicles: List[ContinuousVehicle] = None):
		self.position_is_obstacle = position_is_obstacle
		self.vehicles = []
		if vehicles is not None:
			for v in vehicles:
				self.add_vehicle(v)

	def add_vehicle(self, vehicle: ContinuousVehicle):
		for v in self.vehicles:
			assert v is not vehicle
			v.add_vehicle(vehicle)
			vehicle.add_vehicle(v)
			v._position_is_obstacle = self.position_is_obstacle
		self.vehicles.append(vehicle)

	def remove_vehicle(self, vehicle: ContinuousVehicle):
		delete_index: None | int = None
		for index, v in enumerate(self.vehicles):
			if v is vehicle:
				delete_index = index
		assert delete_index is not None
		for v in self.vehicles:
			v.remove_vehicle(vehicle)

		del self.vehicles[delete_index]

	def roll_forward(self, dt: float):
		for vehicle in self.vehicles:
			vehicle._roll_forward(dt)