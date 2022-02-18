import numpy as np
from enum import Enum
from dataclasses import dataclass

class CellType(Enum):
	road = 0
	civilian = 1
	emergency = 2
	obstacle = 3

@dataclass
class Position:
	x: float
	y: float

	def __add__(self, other):
		return Position(self.x + other.x, self.y + other.y)

	def __sub__(self, other):
		return Position(self.x - other.x, self.y - other.y)

	def distance_to(self, other):
		return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
