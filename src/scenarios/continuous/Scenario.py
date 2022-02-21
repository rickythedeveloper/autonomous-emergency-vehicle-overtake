from dataclasses import dataclass
from typing import List, Callable
from ...utils.Vector2 import Vector2

@dataclass
class VehicleInitializationInfo:
	position: Vector2
	velocity: Vector2

@dataclass
class Scenario:
	civilians: List[VehicleInitializationInfo]
	emergencies: List[VehicleInitializationInfo]
	position_is_in_obstacle: Callable[[Vector2], bool]