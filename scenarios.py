from utils import Position, CellType

# TODO take orientation into account
def is_within_vehicle(relative_position: Position):
    return -1 < relative_position.x < 1 and -2 < relative_position.y < 2

def scenario1(position: Position) -> CellType:
    center = Position(5, 10)
    if position.distance_to(center) < 5:
        return CellType.emergency
    return CellType.road

def scenario2(position: Position) -> CellType:
    if -1 < position.x < 0 or 10 < position.x < 11:
        return CellType.obstacle

    civilian_positions = [Position(2, 15), Position(5, 10)]
    for pos in civilian_positions:
        if is_within_vehicle(position - pos):
            return CellType.civilian

    emergency_position = Position(8, 10)
    if is_within_vehicle(position - emergency_position):
        return CellType.emergency

    return CellType.road
