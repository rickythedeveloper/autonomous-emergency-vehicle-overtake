from utils import Position, CellType
from typing import List, Callable

# TODO take orientation into account
def is_within_vehicle(relative_position: Position):
	return -1 < relative_position.x < 1 and -2 < relative_position.y < 2

def make_scenario(
	civilian_vehicle_positions: List[Position],
	emergency_vehicle_positions: List[Position],
	obstacle_check: Callable[[Position], bool]
) -> Callable[[Position], CellType]:
	def scenario(position: Position) -> CellType:
		for p in civilian_vehicle_positions:
			if is_within_vehicle(p - position):
				return CellType.civilian
		for p in emergency_vehicle_positions:
			if is_within_vehicle(p - position):
				return CellType.emergency
		if obstacle_check(position):
			return CellType.obstacle
		return CellType.road

	return scenario

scenario1 = make_scenario(
	[Position(2, 15), Position(5, 10)],
	[Position(8, 10)],
	lambda position: -1 < position.x < 0 or 10 < position.x < 11
)

scenario2 = make_scenario(
	[Position(2, 20), Position(5, 5)],
	[Position(5, 15)],
	lambda position: -1 < position.x < 0 or 10 < position.x < 11
)

