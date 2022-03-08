from typing import List
from .Scenario import Scenario, VehicleInitializationInfo
from ...utils.Vector2 import Vector2

civilians: List[VehicleInitializationInfo] = [
	# VehicleInitializationInfo(Vector2(0, 10), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(3, 15), Vector2(0, 0)),
	# VehicleInitializationInfo(Vector2(-3, 18), Vector2(0, 0)),
	# VehicleInitializationInfo(Vector2(5, 20), Vector2(0, 0)),
	# VehicleInitializationInfo(Vector2(-5, 22), Vector2(0, 0)),
]

emergencies: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 0), Vector2(0, 0))
]

def position_is_in_obstacle(position: Vector2):
	if position.x < -10: return True
	if position.x > 10: return True
	return False

scenario = Scenario(civilians, emergencies, position_is_in_obstacle)