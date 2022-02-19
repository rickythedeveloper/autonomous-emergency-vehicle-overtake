import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Callable
from utils import CellType, Vector2
from dataclasses import dataclass

@dataclass
class Extent:
	min_x: float
	max_x: float
	min_y: float
	max_y: float

	@property
	def width(self):
		return self.max_x - self.min_x

	@property
	def height(self):
		return self.max_y - self.min_y

def get_cells(scenario: Callable[[Vector2], CellType], size: float, extent: Extent):
	cells = []
	for y in np.arange(extent.min_y, extent.max_y, size):
		row_cells = []
		for x in np.arange(extent.min_x, extent.max_x, size):
			cell_type = scenario(Vector2(x, y))
			row_cells.append(cell_type)
		cells.append(row_cells)
	return cells

Color = Tuple[int, int, int]

def cell_type_to_color(cell_type: CellType) -> Color:
	if cell_type == CellType.road: return 50, 50, 50
	if cell_type == CellType.civilian: return 150, 150, 150
	if cell_type == CellType.emergency: return 150, 0, 0
	if cell_type == CellType.obstacle: return 0, 0, 0
	raise NotImplementedError

def cells_to_colors(cells: List[List[CellType]]) -> List[List[Color]]:
	colors = []
	for row in cells:
		colors_row = []
		for cell in row:
			colors_row.append(cell_type_to_color(cell))
		colors.append(colors_row)
	return colors

def visualise_scenario(scenario: Callable[[Vector2], CellType], cell_size: float, extent: Extent):
	cells = get_cells(scenario, cell_size, extent)
	colors = cells_to_colors(cells)
	plt.imshow(colors, origin='lower', extent=(extent.min_x, extent.max_x, extent.min_y, extent.max_y))
	plt.show()

if __name__ == '__main__':
	pass