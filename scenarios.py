from utils import Vector2, CellType
from typing import List, Callable

# TODO take orientation into account
def is_within_vehicle(relative_position: Vector2):
	return -1 < relative_position.x < 1 and -2 < relative_position.y < 2

def make_scenario(
	civilian_vehicle_positions: List[Vector2],
	emergency_vehicle_positions: List[Vector2],
	obstacle_check: Callable[[Vector2], bool]
) -> Callable[[Vector2], CellType]:
	def scenario(position: Vector2) -> CellType:
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
	[Vector2(2, 15), Vector2(5, 10)],
	[Vector2(8, 10)],
	lambda position: -1 < position.x < 0 or 10 < position.x < 11
)

scenario2 = make_scenario(
	[Vector2(2, 20), Vector2(5, 5)],
	[Vector2(5, 15)],
	lambda position: -1 < position.x < 0 or 10 < position.x < 11
)

