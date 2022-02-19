import copy
from typing import List
from models.discrete.DiscreteVehicle import DiscreteVehicle
from models.discrete.utils import CellType, DiscreteVehicleType, Grid, DiscretePosition

class DiscreteSimulator:
	vehicles: List[DiscreteVehicle]
	grid: Grid

	def __init__(self, grid: Grid, vehicles: List[DiscreteVehicle] = None):
		self.vehicles = []
		self.grid = copy.deepcopy(grid)

		if vehicles is not None:
			for v in vehicles:
				self.add_vehicle(v)
				v.grid = self.grid
		self.update_grid()

	def add_vehicle(self, vehicle: DiscreteVehicle):
		self.vehicles.append(vehicle)
		vehicle._obstacle_positions = self.grid

	def remove_vehicle(self, vehicle: DiscreteVehicle):
		delete_index: None | int = None
		for index, v in enumerate(self.vehicles):
			if v is vehicle:
				delete_index = index
		assert delete_index is not None
		del self.vehicles[delete_index]

	def update_grid(self):
		# reset any vehicles to road
		for i, row in enumerate(self.grid):
			for j, cell in enumerate(row):
				if cell == CellType.emergency or cell == CellType.civilian:
					self.grid[i][j] = CellType.road

		# put vehicles on the grid
		for v in self.vehicles:
			if not v.is_in_grid(): continue
			i, j = v.position
			assert self.grid[i][j] != CellType.emergency and self.grid[i][j] != CellType.civilian # no collisions
			self.grid[i][j] = CellType.civilian if v.vehicle_type == DiscreteVehicleType.civilian else CellType.emergency

	def roll_forward(self):
		for vehicle in self.vehicles:
			vehicle._position = vehicle.compute_next_point()
		self.update_grid()

	def position_is_in_grid(self, position: DiscretePosition):
		if position[0] < 0 or position[1] < 0: return False
		if position[0] >= len(self.grid) or position[1] >= len(self.grid[0]): return False
		return True