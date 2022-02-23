import copy
from typing import List

# imports for simulation
from .models.discrete.DiscreteSimulator import DiscreteSimulator
from .algorithms.discrete.depth_first import DiscreteCivilianVehicle, DiscreteEmergencyVehicle
from .models.discrete.utils import DiscreteSimulationCellType, Grid
from .scenarios.discrete.scenario1 import scenario, width as scenario_width, height as scenario_height

# imports for visualization
from .utils.Vector2 import Vector2
from .visualize.visualize import visualize_result, VisualisationCellType, Extent

def get_visualization_cell_type(position: Vector2, grid: Grid) -> VisualisationCellType:
	cell_coord = (round(position.x), round(position.y))  # cell coordinates

	# if outside the simulation, visualize as obstacle
	if cell_coord[0] < 0 or cell_coord[1] < 0: return VisualisationCellType.obstacle
	if cell_coord[1] >= len(grid): return VisualisationCellType.obstacle
	if cell_coord[0] >= len(grid[0]): return VisualisationCellType.obstacle

	# depending on the simulation cell type, determine the visualization cell type
	sim_cell_type = grid[cell_coord[1]][cell_coord[0]]
	if sim_cell_type == DiscreteSimulationCellType.road: return VisualisationCellType.road
	if sim_cell_type == DiscreteSimulationCellType.civilian: return VisualisationCellType.civilian
	if sim_cell_type == DiscreteSimulationCellType.emergency: return VisualisationCellType.emergency
	if sim_cell_type == DiscreteSimulationCellType.obstacle: return VisualisationCellType.obstacle
	if sim_cell_type == DiscreteSimulationCellType.goal: return VisualisationCellType.goal
	raise NotImplementedError

def discrete_main():
	# given scenario, set up the simulator and the vehicles
	civilians = [DiscreteCivilianVehicle(pos) for pos in scenario.civilian_positions]
	emergencies = [DiscreteEmergencyVehicle(pos) for pos in scenario.emergencie_positions]
	simulator = DiscreteSimulator(scenario.grid, [*civilians, *emergencies])

	# simulate
	n_iter = 15
	data: List[Grid] = [copy.deepcopy(simulator.grid)]
	for i in range(n_iter):
		simulator.roll_forward()
		data.append(copy.deepcopy(simulator.grid))
		print(f'\riter = {i}', end='')

	# visualize
	for i in range(n_iter):
		visualize_result(
			lambda x: get_visualization_cell_type(x, data[i]),
			1,
			Extent(0, scenario_width + 2, 0, scenario_height + 2)
		)

if __name__ == '__main__':
	discrete_main()