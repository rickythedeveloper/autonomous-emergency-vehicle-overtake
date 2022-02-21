from dataclasses import dataclass
from typing import List, Tuple
from ...models.discrete.DiscreteSimulator import DiscreteSimulationCellType

@dataclass
class VehicleInitializationInfo:
	position: Tuple[int, int]

@dataclass
class Scenario:
	civilians: List[VehicleInitializationInfo]
	emergencies: List[VehicleInitializationInfo]
	grid: List[List[DiscreteSimulationCellType]]
