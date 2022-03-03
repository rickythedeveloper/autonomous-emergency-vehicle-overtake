from typing import List
from dataclasses import dataclass
from ...models.discrete.DiscreteVehicle import DiscreteVehicle
from ...models.discrete.utils import DiscretePosition, DiscreteVehicleType, Grid, DiscreteSimulationCellType
from .utils import possible_moves, is_road, is_goal, get_next_position

History = List[DiscretePosition]

def solve_maze(
	grid: Grid,
	current_position: DiscretePosition,
) -> List[DiscretePosition] | None:
	configurations_to_explore: List[History] = []

	for move in possible_moves:
		proposal = get_next_position(current_position, move)
		if is_goal(proposal, grid): return [proposal]
		if is_road(proposal, grid):
			history = [current_position, proposal]
			configurations_to_explore.append(history)

	while len(configurations_to_explore) > 0:
		history = configurations_to_explore[0] # always examine the first one

		for move in possible_moves:
			proposal = get_next_position(history[-1], move)
			if is_goal(proposal, grid): return [*history[1:], proposal]
			if is_road(proposal, grid):
				configurations_to_explore.append([*history, proposal])

		del configurations_to_explore[0]
	return None

class DiscreteCivilianVehicle(DiscreteVehicle):
	def __init__(self, position: DiscretePosition):
		super().__init__(DiscreteVehicleType.civilian, position)

	def compute_next_point(self) -> DiscretePosition:
		return self.position

class DiscreteEmergencyVehicle(DiscreteVehicle):
	path: List[DiscretePosition] | None

	def __init__(self, position: DiscretePosition):
		super().__init__(DiscreteVehicleType.emergency, position)
		self.path = None

	def compute_next_point(self) -> DiscretePosition:
		if self.path is None:
			path = solve_maze(self.grid, self.position)
			if path is not None:
				assert len(path) > 0
				self.path = path
				return path[0]
			return self.position
		else:
			for index, pos in enumerate(self.path):
				if pos != self.position: continue
				if index + 1 < len(self.path): return self.path[index + 1]
				else: return self.position # reached the goal
			raise NotImplementedError