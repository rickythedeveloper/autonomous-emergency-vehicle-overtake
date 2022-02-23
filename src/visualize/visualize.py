from typing import Tuple, List, Callable
from dataclasses import dataclass
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
from ..utils.Vector2 import Vector2

class VisualisationCellType(Enum):
	road = 0
	civilian = 1
	emergency = 2
	obstacle = 3
	goal = 4

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

def get_cells(cell_type_for_position: Callable[[Vector2], VisualisationCellType], size: float, extent: Extent):
	cells = []
	for y in np.arange(extent.min_y, extent.max_y, size):
		row_cells = []
		for x in np.arange(extent.min_x, extent.max_x, size):
			cell_type = cell_type_for_position(Vector2(x, y))
			row_cells.append(cell_type)
		cells.append(row_cells)
	return cells

Color = Tuple[int, int, int]

def cell_type_to_color(cell_type: VisualisationCellType) -> Color:
	if cell_type == VisualisationCellType.road: return 50, 50, 50
	if cell_type == VisualisationCellType.civilian: return 150, 150, 150
	if cell_type == VisualisationCellType.emergency: return 150, 0, 0
	if cell_type == VisualisationCellType.obstacle: return 0, 0, 0
	if cell_type == VisualisationCellType.goal: return 0, 150, 150
	raise NotImplementedError

def cells_to_colors(cells: List[List[VisualisationCellType]]) -> List[List[Color]]:
	colors = []
	for row in cells:
		colors_row = []
		for cell in row:
			colors_row.append(cell_type_to_color(cell))
		colors.append(colors_row)
	return colors

def visualize_result(
	get_visualization_cell_type: Callable[[Vector2], VisualisationCellType],
	cell_size: float,
	extent: Extent
):
	cells = get_cells(get_visualization_cell_type, cell_size, extent)
	colors = cells_to_colors(cells)
	plt.imshow(colors, origin='lower', extent=(extent.min_x, extent.max_x, extent.min_y, extent.max_y))
	plt.show()

if __name__ == '__main__':
	pass