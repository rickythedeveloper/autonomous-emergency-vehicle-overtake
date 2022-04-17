from typing import List, Callable
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

from src.models.continuous.ContinuousVehicle import VehicleType
from src.utils.Vector2 import Vector2
from src.algorithms.continuous.gaussian import weight_density_generator_road_and_vehicle, weight_density_generator_with_road

visualisation_angle = np.pi / 6
STEP = visualisation_angle / 100
def visualise_normalised_gaussian(function: Callable[[float], float]):
	theta = np.arange(-visualisation_angle / 2, visualisation_angle / 2 + STEP, STEP)
	y = [function(t) for t in theta]

	plt.title("Relative likelihood of picking a given angle")
	plt.xlabel("angle")
	plt.ylabel("acceptance probability")
	plt.plot(theta * 180 / np.pi, y, color="green")
	plt.show()

@dataclass
class TestCase:
	traffic_positions: List[Vector2]
	vehicle_types: List[VehicleType]
	road_heading: float
	cone_angle: float

if __name__ == '__main__':
	# traffic_positions = [Vector2(5, 5), Vector2(0, 10), Vector2(5, 15), Vector2(-5, 5), Vector2(0, 10), Vector2(-5, 15)]
	# traffic_positions = [Vector2(1, 1), Vector2(-1, 1), Vector2(1, 2)]
	# traffic_positions = [Vector2(3, 10), Vector2(1, 10), Vector2(5, 15)]
	# traffic_positions = [Vector2(3, 10), Vector2(0, 10), Vector2(5, 15), Vector2(-2, 10), Vector2(-2, 7)]
	# traffic_positions = [Vector2(5, 0)]
	# traffic_positions = [Vector2(-0.1, 5)]
	traffic_positions = [Vector2(3, 7), Vector2(8, 2)] # this is a nice one
	traffic_positions = [Vector2(0.1, 5), Vector2(-3, 3)]  # this represents the bad edge case
	vehicle_types = [VehicleType.civilian for _ in range(len(traffic_positions))]
	road_heading = 0
	cone_angle = np.pi / 6

	gaussian_function = weight_density_generator_road_and_vehicle(traffic_positions, vehicle_types, road_heading, cone_angle)
	# gaussian_function_road = weight_density_generator_with_road(road_heading) # use this to show road gaussian on its own
	visualise_normalised_gaussian(gaussian_function)