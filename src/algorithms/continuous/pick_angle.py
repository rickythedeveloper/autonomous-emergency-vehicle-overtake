from __future__ import annotations

from random import uniform
from typing import Callable

def pick_angle(weight_density_function: Callable[[float], float], max_weight: float, min_angle: float, max_angle: float):
	while True:
		angle = uniform(min_angle, max_angle)
		value = uniform(0, max_weight)
		if value < weight_density_function(angle): return angle