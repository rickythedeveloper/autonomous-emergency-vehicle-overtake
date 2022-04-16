from __future__ import annotations

from typing import Callable, List, Tuple

import numpy as np

from src.algorithms.continuous.utils.heading import clean_heading
from src.models.continuous.ContinuousVehicle import VehicleType
from src.utils.Vector2 import Vector2

GaussianParameter = Tuple[float, float]

def compute_overall_gaussian_parameter(parameters: List[GaussianParameter]) -> GaussianParameter:
	assert len(parameters) > 0
	sum_mu_over_sigma_squared = 0
	sum_one_over_sigma_squared = 0
	for p in parameters:
		mu_i = clean_heading(p[0], -np.pi) # enforce mu_i to be between -pi and pi
		sigma_i = p[1]
		sum_mu_over_sigma_squared += mu_i / (sigma_i ** 2)
		sum_one_over_sigma_squared += 1 / (sigma_i ** 2)
	mu = sum_mu_over_sigma_squared / sum_one_over_sigma_squared
	sigma = np.sqrt(1 / sum_one_over_sigma_squared)
	return mu, sigma

def gaussian_function_generator(parameter: GaussianParameter, normalisation_constant: float) -> Callable[[float], float]:
	def gaussian_function(angle: float) -> float:
		return normalised_angle_gaussian(parameter, angle) * normalisation_constant
	return gaussian_function

def normalised_angle_gaussian(param: GaussianParameter, x: float):
	mu, sigma = param
	x_minus_mu = clean_heading(x - mu, -np.pi) # (x - mu) between -pi and pi
	return np.exp(-0.5 * (x_minus_mu ** 2) / (sigma ** 2))

# parameters for Gaussian angle picking
ROAD_HEADING_SIGMA = np.pi / 16
EMERGENCY_AVOID_SIGMA = np.pi / 36
CIVILIAN_AVOID_SIGMA = np.pi / 24

def weight_density_generator_with_road(road_direction: float):
	return gaussian_function_generator((road_direction, ROAD_HEADING_SIGMA), 1)

def weight_density_generator_road_and_vehicle(traffic_positions: List[Vector2], vehicle_types: List[VehicleType], road_heading: float, cone_angle: float) -> Callable[[float], float]:
	"""Returns the gaussian function given the traffic, road heading and cone angle"""
	gaussian_parameters: List[GaussianParameter] = [
		(road_heading, ROAD_HEADING_SIGMA)  # road heading
	]

	# avoid other vehicles
	for traffic_position, vehicle_type in zip(traffic_positions, vehicle_types):
		mu_opposite = Vector2.zero().heading_to(traffic_position)
		mu_v = clean_heading(mu_opposite + np.pi, -np.pi)

		sigma_v = traffic_position.length * (EMERGENCY_AVOID_SIGMA if vehicle_type == VehicleType.emergency else CIVILIAN_AVOID_SIGMA)
		gaussian_parameters.append((mu_v, sigma_v))

	g_param = compute_overall_gaussian_parameter(gaussian_parameters)

	if g_param[0] < -cone_angle / 2 or g_param[0] > cone_angle / 2:
		left_value = normalised_angle_gaussian(g_param, -cone_angle / 2)
		right_value = normalised_angle_gaussian(g_param, cone_angle / 2)
		normalisation_constant = 1 / max(left_value, right_value)
	else:
		normalisation_constant = 1

	return gaussian_function_generator(g_param, normalisation_constant)