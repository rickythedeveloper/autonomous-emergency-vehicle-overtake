from ..models.continuous.ContinuousVehicle import ContinuousVehicle, VehicleType
from ..utils.Vector2 import Vector2

class ContinuousCivilianVehicle(ContinuousVehicle):
	_width = 2
	_length = 3

	def __init__(self, position: Vector2, velocity: Vector2, heading: float = 0):
		super().__init__(VehicleType.civilian, position, velocity, heading)

	def update_velocity(self):
		self._velocity = Vector2(0.5, 0)

	def contains(self, position: Vector2) -> bool:
		relative_position = position - self.position
		relative_position_vehicle_frame = relative_position.rotated_clockwise(self._heading)
		x_rel, y_rel = relative_position_vehicle_frame.x, relative_position_vehicle_frame.y
		return -self._width / 2 < x_rel < self._width / 2 and -self._length / 2 < y_rel < self._length / 2

class ContinuousEmergencyVehicle(ContinuousVehicle):
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

def position_is_obstacle(position: Vector2) -> bool:
	return -3 < position.y < -2 or 6 < position.y < 7