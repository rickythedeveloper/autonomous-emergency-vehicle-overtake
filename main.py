from utils import Vector2
from models.Simulator import Simulator
from models.vehicles import CivilianVehicle, EmergencyVehicle
from visualise import visualise_scenario, CellType, Extent

def main():
	civilian = CivilianVehicle(Vector2(0, 0), Vector2(0, 0))
	emergency = EmergencyVehicle(Vector2(0, 4), Vector2(0, 0))

	def position_to_cell_type(position: Vector2) -> CellType:
		if civilian.contains(position): return CellType.civilian
		if emergency.contains(position): return CellType.emergency
		return CellType.road

	simulator = Simulator([civilian, emergency])

	dt = 0.5
	total_time = 5
	t = 0
	for i in range(int(total_time / dt)):
		for v in simulator.vehicles:
			v._heading += 0.1
			v.update_velocity()

		simulator.roll_forward(dt)
		t += dt
		visualise_scenario(position_to_cell_type, 0.1, Extent(-5, 5, -5, 10))
		print(f't={t}')

if __name__ == '__main__':
	main()