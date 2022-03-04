from typing import List
from ...models.continuous.ContinuousVehicle import ContinuousVehicle, Control, LateralDirection, VehicleType
from ...utils.Vector2 import Vector2

class ContinuousCivilianVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self):
		super().__init__(VehicleType.civilian)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def compute_control(self) -> Control:
		return Control(2, LateralDirection.left, 50)

class ContinuousEmergencyVehicle(ContinuousVehicle):
	_width = 2
	_length = 3
	collision_test_points_local_frame: List[Vector2] = []

	def __init__(self):
		super().__init__(VehicleType.emergency)

	def contains(self, position: Vector2) -> bool:
		return -self._width / 2 < position.x < self._width / 2 and -self._length / 2 < position.y < self._length / 2

	def compute_control(self) -> Control:
		return Control(1, LateralDirection.right, 50)