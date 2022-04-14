import copy
from typing import List, Callable, Tuple
from dataclasses import dataclass
from datetime import datetime
import os

# imports for simulation
from src.models.continuous.ContinuousVehicle import ContinuousVehicle, VehicleType
from .models.continuous.ContinuousSimulator import ContinuousSimulator, VehicleData
from .algorithms.continuous.algorithm1 import ContinuousCivilianVehicle, ContinuousEmergencyVehicle, VehicleStuckError, Algorithm1Vehicle
from .scenarios.continuous.scenario1 import scenario

# imports for visualization
from .utils.Vector2 import Vector2
from .visualize.visualize import visualize_result, VisualisationCellType, Extent


def get_visualization_cell_type(position: Vector2, vehicles: List[VehicleData]):
	if scenario.position_is_in_obstacle(position): return VisualisationCellType.obstacle
	for v in vehicles:
		if v.object.contains(v.position_world_to_relative(position)):
			if v.object.vehicle_type == VehicleType.emergency: return VisualisationCellType.emergency
			if v.object.vehicle_type == VehicleType.civilian: return VisualisationCellType.civilian

		for future_pose in v.object.future_poses:
			pose_position = v.position_relative_to_world(future_pose.pose.position)
			if (position - pose_position).length < 0.5:
				return VisualisationCellType.emergency if isinstance(v.object, ContinuousEmergencyVehicle) else VisualisationCellType.civilian

	return VisualisationCellType.road

@dataclass
class SimulationSnapshot:
	civilians: List[ContinuousCivilianVehicle]
	emergencies: List[ContinuousEmergencyVehicle]

GOAL_DISTANCE = 100
def continuous_main():
	# given scenario, set up the simulator and the vehicles
	simulator = ContinuousSimulator(
		scenario.position_is_in_obstacle,
		[VehicleData(ContinuousCivilianVehicle(), c.position, c.velocity, 0) for c in scenario.civilians]
		+ [VehicleData(ContinuousEmergencyVehicle(), e.position, e.velocity, 0) for e in scenario.emergencies]
	)

	# simulate
	dt, total_time = 0.5, 50
	n_iter = int(total_time / dt)
	t = 0
	data: List[List[VehicleData]] = []

	# obstacle_maps: List[List[List[Tuple[float, float, float]]]] = []
	iter_done = 0
	emergency_goal_time: float | None = None
	civilian_goal_time: float | None = None
	for i in range(n_iter):
		print(i)
		try:
			simulator.roll_forward(dt)
		except (VehicleStuckError, KeyboardInterrupt):
			print("\nFAIL")
			break

		iter_done = i + 1
		data.append(copy.deepcopy(simulator.vehicles))

		current_time = iter_done * dt
		civilians_have_reached = True
		for vehicle_data in simulator.vehicles:
			if vehicle_data.object.vehicle_type == VehicleType.emergency:
				if vehicle_data.position.y > GOAL_DISTANCE and emergency_goal_time is None:
					emergency_goal_time = current_time
			if vehicle_data.object.vehicle_type == VehicleType.civilian:
				if vehicle_data.position.y < GOAL_DISTANCE:
					civilians_have_reached = False
		if civilians_have_reached and civilian_goal_time is None:
			civilian_goal_time = current_time

		if civilian_goal_time is not None and emergency_goal_time is not None:
			print('\nSUCCESS: stopping simulation because all reached goal')
			break

		# colors: List[List[Tuple[float, float, float]]] = []
		# obstacle_maps.append(colors)
		# for y in np.arange(-15, 15, 1):
		# 	row: List[Tuple[float, float, float]] = []
		# 	colors.append(row)
		# 	for x in np.arange(-15, 15, 1):
		# 		if x == 0 and y == 0:
		# 			color: Tuple[float, float, float] = (0, 0, 0)
		# 		else:
		# 			color: Tuple[float, float, float] = (200, 0, 0) if simulator.vehicles[-1].object.position_will_collide(Vector2(x, y), 0) else (0, 200, 0)
		# 		row.append(color)
		t += dt
		# print(f'\rt={t}', end='')

	total_wasted_proposal_count = 0
	for vehicle_data in simulator.vehicles:
		vehicle: Algorithm1Vehicle = vehicle_data.object
		total_wasted_proposal_count += vehicle.wasted_proposal_count
	ave_closest_distance = sum(simulator.closest_car_to_emergency_distance)/len(simulator.closest_car_to_emergency_distance)
	print(
		f'simulation complete with {iter_done} iterations\n'
		f'E goal: {emergency_goal_time}s\n'
		f'C goal: {civilian_goal_time}s\n'
		f'Wasted proposals: {total_wasted_proposal_count}\n'
		f'min distance average to E: {ave_closest_distance}\n'
		'visualising...'
	)

	# visualize
	save_directory = os.path.join('images', datetime.now().strftime("%Y-%d-%m_%H-%M-%S"))
	for index, snapshot in enumerate(data):
		emergency_vehicle_position: Vector2 | None = None
		for vehicle_data in snapshot:
			if vehicle_data.object.vehicle_type == VehicleType.emergency:
				emergency_vehicle_position = vehicle_data.position
		if emergency_vehicle_position is None: raise Exception('emergency vehicle position not found')

		visualize_result(
			lambda x: get_visualization_cell_type(x, snapshot),
			0.3,
			Extent(
				emergency_vehicle_position.x - 15,
				emergency_vehicle_position.x + 15,
				emergency_vehicle_position.y - 10,
				emergency_vehicle_position.y + 60
			),
			os.path.join(save_directory, 'absolute'),
			str(index)
		)

		# plt.imshow(obstacle_maps[index], origin='lower', extent=(-15, 15, -15, 15))
		# obstacle_dir = os.path.join(save_directory, 'relative')
		# os.makedirs(obstacle_dir, exist_ok=True)
		# plt.savefig(os.path.join(obstacle_dir, str(index)))

if __name__ == '__main__':
	continuous_main()