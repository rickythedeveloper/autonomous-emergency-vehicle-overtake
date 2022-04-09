from typing import List
from .Scenario import Scenario, VehicleInitializationInfo
from ...utils.Vector2 import Vector2

civilians: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 15), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(5, 15), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(10, 12), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(0, 30), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(3, 35), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(-3, 38), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(5, 40), Vector2(0, 0)),
	VehicleInitializationInfo(Vector2(-5, 42), Vector2(0, 0)),
]

emergencies: List[VehicleInitializationInfo] = [
	VehicleInitializationInfo(Vector2(0, 0), Vector2(0, 0))
]

def position_is_in_obstacle(position: Vector2):
	if position.x < -10: return True
	if position.x > 10: return True
	return False

scenario = Scenario(civilians, emergencies, position_is_in_obstacle)