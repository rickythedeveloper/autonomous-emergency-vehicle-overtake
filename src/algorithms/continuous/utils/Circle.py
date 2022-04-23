from dataclasses import dataclass
from ....utils.Vector2 import Vector2

@dataclass
class Circle:
	center: Vector2
	radius: float