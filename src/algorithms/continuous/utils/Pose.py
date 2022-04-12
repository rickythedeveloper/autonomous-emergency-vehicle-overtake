from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ....utils.Vector2 import Vector2

@dataclass
class Pose:
	position: Vector2
	_heading: float

	def __init__(self, position: Vector2, heading: float):
		self.position = position
		self.heading = heading

	def __str__(self):
		return f'Pose(x={round(self.position.x, 2)}, y={round(self.position.y)}, heading={self.heading}rad ({round(self.heading * 180 / np.pi)}deg))'

	@property
	def heading(self): return self._heading

	@heading.setter
	def heading(self, value: float): self._heading = value % (2 * np.pi)

	@staticmethod
	def zero() -> Pose: return Pose(Vector2(0, 0), 0)

	def position_world_to_relative(self, position: Vector2) -> Vector2:
		return (position - self.position).rotated_clockwise(-self.heading)

	def position_relative_to_world(self, position: Vector2) -> Vector2:
		return position.rotated_clockwise(self.heading) + self.position