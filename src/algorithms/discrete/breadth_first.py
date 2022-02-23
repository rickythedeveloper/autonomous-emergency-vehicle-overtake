from typing import List
from dataclasses import dataclass
from ...models.discrete.DiscreteVehicle import DiscreteVehicle
from ...models.discrete.utils import DiscretePosition, DiscreteVehicleType, Grid, DiscreteSimulationCellType
from .utils import possible_moves, is_road, is_goal, get_next_position

@dataclass
class StartingConfiguration:
	start: DiscretePosition
	history: List[DiscretePosition]

def solve_maze(
	grid: Grid,
	current_position: DiscretePosition,
) -> List[DiscretePosition] | None:
	configurations_to_explore: List[StartingConfiguration] = []

	for move in possible_moves:
		proposal = get_next_position(current_position, move)
		if is_goal(proposal, grid): return [proposal]
		if is_road(proposal, grid):
			configurations_to_explore.append(StartingConfiguration(proposal, [current_position]))

	while len(configurations_to_explore) > 0:
		config = configurations_to_explore[0]

		for move in possible_moves:
			proposal = get_next_position(config.start, move)
			if is_goal(proposal, grid): return [*config.history[1:], proposal]
			if is_road(proposal, grid):
				configurations_to_explore.append(StartingConfiguration(proposal, [*config.history, config.start]))

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