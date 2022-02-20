from __future__ import annotations # allows referencing Vector2 from inside Vector2
import numpy as np
from dataclasses import dataclass

@dataclass
class Vector2:
	x: float
	y: float

	def __add__(self, other: Vector2):
		if not isinstance(other, Vector2):
			raise NotImplementedError
		return Vector2(self.x + other.x, self.y + other.y)

	def __sub__(self, other: Vector2):
		if not isinstance(other, Vector2):
			raise NotImplementedError
		return Vector2(self.x - other.x, self.y - other.y)

	def __mul__(self, scalar: int | float):
		if not (isinstance(scalar, int) or isinstance(scalar, float)):
			raise NotImplementedError
		return Vector2(self.x * scalar, self.y * scalar)

	def __truediv__(self, scalar: int | float):
		return Vector2(self.x / scalar, self.y / scalar)

	def distance_to(self, other):
		return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

	def rotated_clockwise(self, angle: float) -> Vector2:
		sine, cosine = np.sin(angle), np.cos(angle)
		x = self.x * cosine - self.y * sine
		y = self.x * sine + self.y * cosine
		return Vector2(x, y)
