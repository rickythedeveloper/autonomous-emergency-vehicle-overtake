from __future__ import annotations

from typing import List

import numpy as np

from src.utils.Vector2 import Vector2

def test_points_on_box(width: float, length: float, spacing: float = 0.1) -> List[Vector2]:
	half_width, half_length = width / 2, length / 2
	test_points = []
	# front and back
	for y in [-half_length, half_length]:
		for x in np.arange(-half_width, half_length, spacing):
			test_points.append(Vector2(x, y))

	# both sides
	for x in [-half_width, half_width]:
		for y in np.arange(-half_length, half_length, spacing):
			test_points.append(Vector2(x, y))

	return test_points