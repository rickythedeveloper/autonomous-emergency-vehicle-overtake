from enum import Enum
from ...models.discrete.utils import DiscretePosition, Grid, DiscreteSimulationCellType

class MoveType(Enum):
	up = 0
	down = 1
	left = 2
	right = 3

possible_moves = [MoveType.up, MoveType.left, MoveType.right, MoveType.down]

def is_road(position: DiscretePosition, grid: Grid):
	return grid[position.y][position.x] == DiscreteSimulationCellType.road

def is_goal(position: DiscretePosition, grid: Grid):
	return grid[position.y][position.x] == DiscreteSimulationCellType.goal

def get_next_position(current_position: DiscretePosition, move: MoveType) -> DiscretePosition:
	if move == MoveType.up: return DiscretePosition(current_position.x, current_position.y + 1)
	if move == MoveType.down: return DiscretePosition(current_position.x, current_position.y - 1)
	if move == MoveType.left: return DiscretePosition(current_position.x - 1, current_position.y)
	if move == MoveType.right: return DiscretePosition(current_position.x + 1, current_position.y)
	raise NotImplementedError