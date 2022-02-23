from .Scenario import Scenario
from ...models.discrete.DiscreteSimulator import DiscreteSimulationCellType
from ...models.discrete.utils import DiscretePosition

civilians = [
	DiscretePosition(0, 0),
	DiscretePosition(0, 3),
	DiscretePosition(1, 4),
	DiscretePosition(2, 2),
	DiscretePosition(2, 4),
	DiscretePosition(3, 4),
	DiscretePosition(4, 1),
]

emergencies = [
	DiscretePosition(2, 0)
]

width, height = 5, 6
grid = [[DiscreteSimulationCellType.road for _ in range(width)] for _ in range(height)]

# the top row is the goal
for i in range(width):
	grid[height - 1][i] = DiscreteSimulationCellType.goal

scenario = Scenario(civilians, emergencies, grid)