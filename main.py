from visualise import visualise_scenario, Extent
from scenarios import scenario1, scenario2

def main():
	min_x, max_x, min_y, max_y = -5, 15, 0, 30
	extent = Extent(min_x, max_x, min_y, max_y)
	cell_size = 0.1
	visualise_scenario(scenario1, cell_size, extent)

if __name__ == '__main__':
	main()