import copy
from typing import List
from dataclasses import dataclass
import numpy as np

# imports for simulation
from src.models.continuous.ContinuousVehicle import ContinuousVehicle, VehicleType
from .models.continuous.ContinuousSimulator import ContinuousSimulator, VehicleData
from .algorithms.continuous.algorithm1 import ContinuousCivilianVehicle, ContinuousEmergencyVehicle
from .scenarios.continuous.scenario1 import scenario

# imports for visualization
from .utils.Vector2 import Vector2
from .visualize.visualize import visualize_result, VisualisationCellType, Extent


def get_visualization_cell_type(position: Vector2, vehicles: List[VehicleData]):
	if scenario.position_is_in_obstacle(position): return VisualisationCellType.obstacle
	for v in vehicles:
		if v.object.contains(v.position_world_to_relative(position)):
			if v.object.vehicle_type == VehicleType.emergency: return VisualisationCellType.emergency
			if v.object.vehicle_type == VehicleType.civilian: return VisualisationCellType.civilian

		if isinstance(v.object, ContinuousEmergencyVehicle):
			for pose in v.object.future_poses:
				pose_position = v.position_relative_to_world(pose.position)
				if (position - pose_position).length < 1: return VisualisationCellType.emergency

	return VisualisationCellType.road

@dataclass
class SimulationSnapshot:
	civilians: List[ContinuousCivilianVehicle]
	emergencies: List[ContinuousEmergencyVehicle]

def continuous_main():
	# given scenario, set up the simulator and the vehicles
	simulator = ContinuousSimulator(
		scenario.position_is_in_obstacle,
		[VehicleData(ContinuousCivilianVehicle(), c.position, c.velocity, 0) for c in scenario.civilians]
		+ [VehicleData(ContinuousEmergencyVehicle(), e.position, e.velocity, 0) for e in scenario.emergencies]
	)

	# simulate
	dt, total_time = 0.5, 50
	n_iter = int(total_time / dt)
	t = 0
	data: List[List[VehicleData]] = []
	for i in range(n_iter):
		simulator.roll_forward(dt)
		data.append(copy.deepcopy(simulator.vehicles))
		t += dt
		# print(f'\rt={t}', end='')

	# visualize
	for snapshot in data:
		visualize_result(
			lambda x: get_visualization_cell_type(x, snapshot),
			0.3,
			Extent(-20, 20, -10, 60)
		)

if __name__ == '__main__':
	continuous_main()