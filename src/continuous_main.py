import copy
from typing import List
from dataclasses import dataclass

# imports for simulation
from .models.continuous.ContinuousSimulator import ContinuousSimulator
from .algorithms.continuous.algorithm1 import ContinuousCivilianVehicle, ContinuousEmergencyVehicle
from .scenarios.continuous.scenario1 import scenario

# imports for visualization
from .utils.Vector2 import Vector2
from .visualize.visualize import visualize_result, VisualisationCellType, Extent

def get_visualization_cell_type(
	position: Vector2,
	civilians: List[ContinuousCivilianVehicle],
	emergencies: List[ContinuousEmergencyVehicle]
) -> VisualisationCellType:
	if scenario.position_is_in_obstacle(position): return VisualisationCellType.obstacle
	for civilian in civilians:
		if civilian.contains(position): return VisualisationCellType.civilian
	for emergency in emergencies:
		if emergency.contains(position): return VisualisationCellType.emergency
	return VisualisationCellType.road

@dataclass
class SimulationSnapshot:
	civilians: List[ContinuousCivilianVehicle]
	emergencies: List[ContinuousEmergencyVehicle]

def continuous_main():
	# given scenario, set up the simulator and the vehicles
	civilians = [ContinuousCivilianVehicle(c.position, c.velocity) for c in scenario.civilians]
	emergencies = [ContinuousEmergencyVehicle(e.position, e.velocity) for e in scenario.emergencies]
	simulator = ContinuousSimulator(scenario.position_is_in_obstacle, [*civilians, *emergencies])

	# simulate
	dt, total_time = 0.5, 5
	n_iter = int(total_time / dt)
	t = 0
	data: List[SimulationSnapshot] = []
	for i in range(n_iter):
		for v in simulator.vehicles:
			v._heading += 0.1 # included as an example
			v.update_velocity()

		simulator.roll_forward(dt)
		data.append(SimulationSnapshot(copy.deepcopy(civilians), copy.deepcopy(emergencies)))
		t += dt
		print(f'\rt={t}', end='')

	# visualize
	for i in range(n_iter):
		snapshot = data[i]
		visualize_result(
			lambda x: get_visualization_cell_type(x, snapshot.civilians, snapshot.emergencies),
			0.1,
			Extent(-5, 5, -5, 10)
		)

if __name__ == '__main__':
	continuous_main()