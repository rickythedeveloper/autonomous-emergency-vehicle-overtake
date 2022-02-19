import numpy as np
from models.Vehicle import Vehicle, VehicleType
from utils import Vector2

class CivilianVehicle(Vehicle):
	_width = 2
	_length = 3

	def __init__(self, position: Vector2, velocity: Vector2, heading: float = 0):
		super().__init__(VehicleType.civilian, position, velocity, heading)

	def update_velocity(self):
		self._velocity = Vector2(0.5, 0)

	@property
	def top_left_position(self):
		sine = np.sin(self._heading)
		cosine = np.cos(self._heading)
		relative_position = Vector2(
			-self._width * cosine + self._length * sine,
			self._width * sine + self._length * cosine
		) / 2
		return self.position + relative_position

	@property
	def bottom_right_position(self):
		sine = np.sin(self._heading)
		cosine = np.cos(self._heading)
		relative_position = Vector2(
			self._width * cosine - self._length * sine,
			-self._width * sine - self._length * cosine
		) / 2
		return self.position + relative_position

	def contains(self, position: Vector2) -> bool:
		relative_position = position - self.position
		relative_position_vehicle_frame = relative_position.rotated_clockwise(self._heading)
		x_rel, y_rel = relative_position_vehicle_frame.x, relative_position_vehicle_frame.y
		return -self._width / 2 < x_rel < self._width / 2 and -self._length / 2 < y_rel < self._length / 2

class EmergencyVehicle(Vehicle):
	_width = 2
	_length = 3

	def __init__(self, position: Vector2, velocity: Vector2, heading: float = 0):
		super().__init__(VehicleType.emergency, position, velocity, heading)

	def update_velocity(self):
		self._velocity = Vector2(1, 0)

	def contains(self, position: Vector2) -> bool:
		relative_position = position - self.position
		relative_position_vehicle_frame = relative_position.rotated_clockwise(self._heading)
		x_rel, y_rel = relative_position_vehicle_frame.x, relative_position_vehicle_frame.y
		return -self._width / 2 < x_rel < self._width / 2 and -self._length / 2 < y_rel < self._length / 2