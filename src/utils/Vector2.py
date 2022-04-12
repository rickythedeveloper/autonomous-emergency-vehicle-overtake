from __future__ import annotations # allows referencing Vector2 from inside Vector2
import numpy as np
from dataclasses import dataclass

@dataclass
class Vector2:
	x: float
	y: float

	@staticmethod
	def zero() -> Vector2: return Vector2(0, 0)

	@staticmethod
	def from_heading(heading: float) -> Vector2:
		return Vector2(np.sin(heading), np.cos(heading))

	def __add__(self, other: Vector2):
		if not isinstance(other, Vector2):
			raise NotImplementedError
		return Vector2(self.x + other.x, self.y + other.y)

	def __sub__(self, other: Vector2):
		if not isinstance(other, Vector2):
			raise NotImplementedError
		return Vector2(self.x - other.x, self.y - other.y)

	def __mul__(self, scalar: int | float):
		return Vector2(self.x * scalar, self.y * scalar)

	def __truediv__(self, scalar: int | float):
		return Vector2(self.x / scalar, self.y / scalar)

	def distance_to(self, other):
		return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

	def heading_to(self, other: Vector2) -> float:
		rel: Vector2 = other - self
		angle = np.arctan(rel.x / rel.y) # between -pi/2 and pi/2
		if rel.x < 0 and rel.y > 0: angle += 2 * np.pi
		elif rel.y < 0: angle += np.pi
		assert 0 <= angle <= 2 * np.pi, f'heading {angle} from {self} to {other} is out of range'
		return angle

	def rotated_clockwise(self, angle: float) -> Vector2:
		sine, cosine = np.sin(angle), np.cos(angle)
		x = self.x * cosine + self.y * sine
		y = -self.x * sine + self.y * cosine
		return Vector2(x, y)

	@property
	def np_array(self) -> np.ndarray:
		return np.array([self.x, self.y])

	@property
	def slope(self):
		if np.abs(self.x) < 1e-5: return np.inf
		return self.y / self.x

	@property
	def length(self):
		return np.sqrt(self.x ** 2 + self.y ** 2)
