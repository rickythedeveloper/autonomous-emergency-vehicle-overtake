from enum import Enum
from typing import List, Callable
from ..utils.Vector2 import Vector2

class VisualisationCellType(Enum):
	road = 0
	civilian = 1
	emergency = 2
	obstacle = 3

# TODO take orientation into account
# TODO use the simulation info to work this out instead of defining it here
def is_within_vehicle(relative_position: Vector2):
	return -1 < relative_position.x < 1 and -2 < relative_position.y < 2

def make_scenario(
	civilian_vehicle_positions: List[Vector2],
	emergency_vehicle_positions: List[Vector2],
	obstacle_check: Callable[[Vector2], bool]
) -> Callable[[Vector2], VisualisationCellType]:
	def scenario(position: Vector2) -> VisualisationCellType:
		for p in civilian_vehicle_positions:
			if is_within_vehicle(p - position):
				return VisualisationCellType.civilian
		for p in emergency_vehicle_positions:
			if is_within_vehicle(p - position):
				return VisualisationCellType.emergency
		if obstacle_check(position):
			return VisualisationCellType.obstacle
		return VisualisationCellType.road

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

