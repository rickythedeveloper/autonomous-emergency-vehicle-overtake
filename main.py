from utils import Vector2
from visualise import visualise_scenario, CellType, Extent
from models.discrete.DiscreteSimulator import DiscreteSimulator
from models.discrete.vehicles import CivilianVehicle, EmergencyVehicle
from models.discrete.utils import CellType as SimCellType

def main():
	civilian = CivilianVehicle((1, 2))
	emergency = EmergencyVehicle((3, 2))
	grid = []
	for i in range(15):
		grid.append([])
		for j in range(15):
			grid[i].append(SimCellType.road)

	simulator = DiscreteSimulator(grid, [civilian, emergency])

	def position_to_cell_type(position: Vector2) -> CellType:
		grid_position = (round(position.x), round(position.y))

		if grid_position[0] < 0 or grid_position[1] < 0: return CellType.obstacle
		if grid_position[0] >= len(simulator.grid): return CellType.obstacle
		if grid_position[1] >= len(simulator.grid[0]): return CellType.obstacle

		sim_cell_type = simulator.grid[grid_position[0]][grid_position[1]]
		if sim_cell_type == SimCellType.road: return CellType.road
		if sim_cell_type == SimCellType.civilian: return CellType.civilian
		if sim_cell_type == SimCellType.emergency: return CellType.emergency
		if sim_cell_type == SimCellType.obstacle: return CellType.obstacle

	n_iter = 15
	for i in range(n_iter):
		simulator.roll_forward()
		visualise_scenario(position_to_cell_type, 1, Extent(-1, 20, -1, 20))

if __name__ == '__main__':
	main()