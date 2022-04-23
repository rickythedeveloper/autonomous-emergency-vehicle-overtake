import numpy as np

def clean_heading(heading: float, min_value: float = 0) -> float:
	"""Compute the value of heading in the range of [min_value, min_value + 2pi)"""
	max_value = min_value + 2 * np.pi
	h = heading
	while h < min_value: h += 2 * np.pi
	while h >= max_value: h -= 2 * np.pi
	return h

def subtract_heading(h1: float, h2: float, min_value: float) -> float:
	"""Calculate heading 1 minus heading 2 in the range of [min_value, min_value + 2 pi]"""
	diff = h1 - h2
	return clean_heading(diff, min_value)