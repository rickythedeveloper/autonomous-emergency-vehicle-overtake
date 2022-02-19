from typing import List
from models.Vehicle import Vehicle

class Simulator:
	vehicles: List[Vehicle]

	def __init__(self, vehicles: List[Vehicle] = None):
		self.vehicles = []
		if vehicles is not None:
			for v in vehicles:
				self.add_vehicle(v)

	def add_vehicle(self, vehicle: Vehicle):
		for v in self.vehicles:
			assert v is not vehicle
			v.add_vehicle(vehicle)
			vehicle.add_vehicle(v)
		self.vehicles.append(vehicle)

	def remove_vehicle(self, vehicle: Vehicle):
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