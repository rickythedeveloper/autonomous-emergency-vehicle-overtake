from utils import Extent
from visualise import visualise_scenario
from scenarios import scenario1, scenario2

def main():
    min_x, max_x, min_y, max_y = -5, 15, 0, 30
    extent = Extent(min_x, max_x, min_y, max_y)
    cell_size = 0.2
    visualise_scenario(scenario2, cell_size, extent)

if __name__ == '__main__':
    main()