from .Scenario import Scenario, VehicleInitializationInfo
from ...models.discrete.DiscreteSimulator import DiscreteSimulationCellType

civilians = [
	VehicleInitializationInfo((1, 5))
]

emergencies = [
	VehicleInitializationInfo((5, 2))
]

grid = [[DiscreteSimulationCellType.road for _ in range(15)] for _ in range(15)]

scenario = Scenario(civilians, emergencies, grid)