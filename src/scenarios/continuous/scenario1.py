from typing import List
from .Scenario import Scenario, VehicleInitializationInfo
from ...utils.Vector2 import Vector2

civilians: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 0), Vector2(0, 0))
]

emergencies: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 4), Vector2(0, 0))
]

def position_is_in_obstacle(position: Vector2):
	if 7 < position.y < 8: return True
	if -4 < position.y < -3: return True
	return False

scenario = Scenario(civilians, emergencies, position_is_in_obstacle)