import copy
from dataclasses import dataclass
from typing import List, Tuple
from ...models.discrete.DiscreteSimulator import DiscreteSimulationCellType, DiscretePosition

class Scenario:
	civilian_positions: List[DiscretePosition]
	emergencie_positions: List[DiscretePosition]
	grid: List[List[DiscreteSimulationCellType]]

	def __init__(
		self,
		civilians: List[DiscretePosition],
		emergencies: List[DiscretePosition],
		grid: List[List[DiscreteSimulationCellType]]
	):
		# surround the scenario with obstacle so the vehicles do not leave the grid
		civilians_copy = copy.deepcopy(civilians)
		for c in civilians_copy:
			c.x += 1
			c.y += 1
		emergencies_copy = copy.deepcopy(emergencies)
		for e in emergencies_copy:
			e.x += 1
			e.y += 1
		grid_copy = copy.deepcopy(grid)
		original_width = len(grid_copy[0])
		for row in grid_copy:
			row.insert(0, DiscreteSimulationCellType.obstacle)
			row.append(DiscreteSimulationCellType.obstacle)
		grid_copy.insert(0, [DiscreteSimulationCellType.obstacle for _ in range(original_width + 2)])
		grid_copy.append([DiscreteSimulationCellType.obstacle for _ in range(original_width + 2)])

		self.civilian_positions = civilians_copy
		self.emergencie_positions = emergencies_copy
		self.grid = grid_copy