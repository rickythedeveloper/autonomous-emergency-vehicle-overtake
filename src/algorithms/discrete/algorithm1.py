from ...models.discrete.DiscreteVehicle import DiscreteVehicle
from ...models.discrete.utils import DiscretePosition, DiscreteVehicleType

class DiscreteCivilianVehicle(DiscreteVehicle):
	def __init__(self, position: DiscretePosition):
		super().__init__(DiscreteVehicleType.civilian, position)

	def compute_next_point(self) -> DiscretePosition:
		return self.position[0], self.position[1] + 1

class DiscreteEmergencyVehicle(DiscreteVehicle):
	def __init__(self, position: DiscretePosition):
		super().__init__(DiscreteVehicleType.emergency, position)

	def compute_next_point(self) -> DiscretePosition:
		return self.position[0] + 1, self.position[1]