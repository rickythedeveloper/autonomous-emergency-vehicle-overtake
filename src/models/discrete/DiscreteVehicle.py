from abc import ABC, abstractmethod
from .utils import DiscretePosition, DiscreteVehicleType, Grid

class DiscreteVehicle(ABC):
	_vehicle_type: DiscreteVehicleType
	_position: DiscretePosition
	grid: Grid

	def __init__(self, vehicle_type: DiscreteVehicleType, position: DiscretePosition):
		self._vehicle_type = vehicle_type
		self._position = position

	@property
	def vehicle_type(self): return self._vehicle_type

	@property
	def position(self): return self._position

	def is_in_grid(self):
		if self.position.x < 0 or self.position.y < 0: return False
		if self.position.y >= len(self.grid) or self.position.x >= len(self.grid[0]): return False
		return True

	@abstractmethod
	def compute_next_point(self) -> DiscretePosition:
		"""
		This function should update the velocity property based on the information the vehicle holds.
		"""
		raise NotImplementedError
