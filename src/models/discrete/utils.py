from typing import Tuple, List
from enum import Enum

DiscretePosition = Tuple[int, int]

class DiscreteSimulationCellType(Enum):
	civilian = 0
	emergency = 1
	obstacle = 2
	road = 3

class DiscreteVehicleType(Enum):
	civilian = 0
	emergency = 1

Grid = List[List[DiscreteSimulationCellType]]