from typing import List
from .Scenario import Scenario, VehicleInitializationInfo
from ...utils.Vector2 import Vector2

civilians: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 0), Vector2(0, 0))
]

emergencies: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(2, 0), Vector2(0, 0))
]

def position_is_in_obstacle(position: Vector2):
	return False

scenario = Scenario(civilians, emergencies, position_is_in_obstacle)