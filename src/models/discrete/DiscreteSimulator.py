import copy
from typing import List
from .DiscreteVehicle import DiscreteVehicle
from .utils import DiscreteSimulationCellType, DiscreteVehicleType, Grid, DiscretePosition

class DiscreteSimulator:
	vehicles: List[DiscreteVehicle]
	grid: Grid

	def __init__(self, grid: Grid, vehicles: List[DiscreteVehicle] = None):
		self.vehicles = []
		self.grid = copy.deepcopy(grid)

		if vehicles is not None:
			for v in vehicles:
				self.add_vehicle(v)
		self.update_grid()

	def add_vehicle(self, vehicle: DiscreteVehicle):
		self.vehicles.append(vehicle)
		vehicle.grid = self.grid

	def remove_vehicle(self, vehicle: DiscreteVehicle):
		delete_index: None | int = None
		for index, v in enumerate(self.vehicles):
			if v is vehicle:
				delete_index = index
		assert delete_index is not None
		del self.vehicles[delete_index]

	def update_grid(self):
		# reset any vehicles to road
		for j, row in enumerate(self.grid):
			for i, cell in enumerate(row):
				if cell == DiscreteSimulationCellType.emergency or cell == DiscreteSimulationCellType.civilian:
					self.grid[j][i] = DiscreteSimulationCellType.road

		# put vehicles on the grid
		for v in self.vehicles:
			if not v.is_in_grid(): continue
			i, j = v.position
			assert self.grid[j][i] != DiscreteSimulationCellType.emergency # no collisions
			assert self.grid[j][i] != DiscreteSimulationCellType.civilian # no collisions
			self.grid[j][i] = \
				DiscreteSimulationCellType.civilian if v.vehicle_type == DiscreteVehicleType.civilian \
				else DiscreteSimulationCellType.emergency

	def roll_forward(self):
		for vehicle in self.vehicles:
			vehicle._position = vehicle.compute_next_point()
		self.update_grid()

	def position_is_in_grid(self, position: DiscretePosition):
		if position[0] < 0 or position[1] < 0: return False
		if position[1] >= len(self.grid) or position[0] >= len(self.grid[0]): return False
		return True