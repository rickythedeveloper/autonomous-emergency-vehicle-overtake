from .utils.Vector2 import Vector2
from .visualize.visualize import visualise_scenario, VisualisationCellType, Extent

from .models.discrete.DiscreteSimulator import DiscreteSimulator
from src.configs.config2 import DiscreteCivilianVehicle, DiscreteEmergencyVehicle
from .models.discrete.utils import DiscreteSimulationCellType

from .models.continuous.ContinuousSimulator import ContinuousSimulator
from .configs.config1 import ContinuousCivilianVehicle, ContinuousEmergencyVehicle, position_is_obstacle

def run_config1():
	civilians = [ContinuousCivilianVehicle(Vector2(0, 0), Vector2(0, 0))]
	emergencies = [ContinuousEmergencyVehicle(Vector2(0, 4), Vector2(0, 0))]

	def visualisation_cell_type_for_position(position: Vector2) -> VisualisationCellType:
		if position_is_obstacle(position): return VisualisationCellType.obstacle
		for c in civilians:
			if c.contains(position): return VisualisationCellType.civilian
		for e in emergencies:
			if e.contains(position): return VisualisationCellType.emergency
		return VisualisationCellType.road

	simulator = ContinuousSimulator(position_is_obstacle, [*civilians, *emergencies])

	dt = 0.5
	total_time = 5
	t = 0
	for i in range(int(total_time / dt)):
		for v in simulator.vehicles:
			v._heading += 0.1
			v.update_velocity()

		simulator.roll_forward(dt)
		t += dt
		visualise_scenario(visualisation_cell_type_for_position, 0.1, Extent(-5, 5, -5, 10))
		print(f't={t}')

def run_config2():
	civilians = [DiscreteCivilianVehicle((1, 5))]
	emergencies = [DiscreteEmergencyVehicle((5, 2))]
	grid = []
	for j in range(15):
		grid.append([])
		for i in range(15):
			grid[j].append(DiscreteSimulationCellType.road)

	simulator = DiscreteSimulator(grid, [*civilians, *emergencies])

	def visualisation_cell_type_for_position(position: Vector2) -> VisualisationCellType:
		grid_position = (round(position.x), round(position.y))

		if grid_position[0] < 0 or grid_position[1] < 0: return VisualisationCellType.obstacle
		if grid_position[1] >= len(simulator.grid): return VisualisationCellType.obstacle
		if grid_position[0] >= len(simulator.grid[0]): return VisualisationCellType.obstacle

		sim_cell_type = simulator.grid[grid_position[1]][grid_position[0]]
		if sim_cell_type == DiscreteSimulationCellType.road: return VisualisationCellType.road
		if sim_cell_type == DiscreteSimulationCellType.civilian: return VisualisationCellType.civilian
		if sim_cell_type == DiscreteSimulationCellType.emergency: return VisualisationCellType.emergency
		if sim_cell_type == DiscreteSimulationCellType.obstacle: return VisualisationCellType.obstacle
		raise NotImplementedError

	n_iter = 15
	for i in range(n_iter):
		simulator.roll_forward()
		visualise_scenario(visualisation_cell_type_for_position, 1, Extent(-1, 20, -1, 20))

CONFIG_NUMBER = 1
def main():
	if CONFIG_NUMBER == 1: run_config1()
	elif CONFIG_NUMBER == 2: run_config2()
	else: NotImplementedError

if __name__ == '__main__':
	main()