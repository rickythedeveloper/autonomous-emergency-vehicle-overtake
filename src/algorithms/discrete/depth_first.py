from typing import List
from ...models.discrete.DiscreteVehicle import DiscreteVehicle
from ...models.discrete.utils import DiscretePosition, DiscreteVehicleType, Grid, DiscreteSimulationCellType
from .utils import possible_moves, is_road, is_goal, get_next_position

def solve_maze(
	grid: Grid,
	current_position: DiscretePosition,
	history: List[DiscretePosition]
) -> List[DiscretePosition] | None:
	for move in possible_moves:
		proposal = get_next_position(current_position, move)

		# if goal, return the proposal
		if is_goal(proposal, grid):
			return [proposal]

		# if the vehicle has been to the proposal, ignore
		has_been_there = False
		for cell in history:
			if cell == proposal:
				has_been_there = True
				break
		if has_been_there: continue

		# if the proposal is road, try find a path from there
		if is_road(proposal, grid):
			result = solve_maze(grid, proposal, [*history, current_position])
			if result is not None:
				return [proposal, *result]
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
			path = solve_maze(self.grid, self.position, [])
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