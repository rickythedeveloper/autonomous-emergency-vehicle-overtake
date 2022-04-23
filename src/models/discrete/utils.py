from typing import List
from enum import Enum
from dataclasses import dataclass

@dataclass
class DiscretePosition:
	x: int
	y: int

class DiscreteSimulationCellType(Enum):
	civilian = 0
	emergency = 1
	obstacle = 2
	road = 3
	goal = 4

class DiscreteVehicleType(Enum):
	civilian = 0
	emergency = 1

Grid = List[List[DiscreteSimulationCellType]]