import math
from utilities.game import FIELD_LENGTH
from dataclasses import dataclass
from wpimath.geometry import Translation2d
import astar

Point = tuple[float, float]


# borrowed from https://stackoverflow.com/a/9997374
def ccw(A: Point, B: Point, C: Point):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


# Return true if line segments AB and CD intersect
def intersect(A: Point, B: Point, C: Point, D: Point):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


@dataclass
class Rect:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def line_intersect(self, A: Point, B: Point) -> bool:
        return (
            intersect(A, B, (self.x_min, self.y_min), (self.x_max, self.y_min))
            or intersect(A, B, (self.x_max, self.y_min), (self.x_max, self.y_max))
            or intersect(A, B, (self.x_max, self.y_max), (self.x_min, self.y_max))
            or intersect(A, B, (self.x_min, self.y_max), (self.x_min, self.y_min))
        )

    def flip(self):
        return Rect(
            FIELD_LENGTH - self.x_min, self.y_min, FIELD_LENGTH - self.x_max, self.y_max
        )

    def expand(self, amount):
        self.x_min -= amount
        self.y_min -= amount
        self.x_max += amount
        self.y_max += amount


ROBOT_LEN = 1.0105
ROBOT_WIDTH = 0.8705

CHARGE_STATION = Rect(2.925, 1.527, 4.859, 3.947)
CHARGE_STATION.expand(ROBOT_LEN / 2)
DIVIDER = Rect(0, 5.45, 3.36, 5.5)
DIVIDER.expand(ROBOT_WIDTH / 2)

OBSTACLES = [CHARGE_STATION, CHARGE_STATION.flip(), DIVIDER, DIVIDER.flip()]


def is_visible(p1: Point, p2: Point):
    return all(not ob.line_intersect(p1, p2) for ob in OBSTACLES)


waypoints_blue = [
    (2.600, 0.754),
    (5.000, 0.754),
    (2.600, 4.732),
    (5.000, 4.732),
    (3.700, 6.118),
]
waypoints_red = []
for w in waypoints_blue:
    waypoints_red.append((FIELD_LENGTH - w[0], [1]))

waypoints: list[Point] = [*waypoints_blue, *waypoints_red]


def _find_path(start: Point, goal: Point) -> list[Point]:
    new_waypoints = waypoints + [start, goal]
    return astar.find_path(
        start,
        goal,
        neighbors_fnct=lambda p: [x for x in new_waypoints if is_visible(x, p)],
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
        distance_between_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
    )


def find_path(start: Translation2d, end: Translation2d) -> list[Translation2d]:
    path = _find_path((start.x, start.y), (end.x, end.y))
    return [Translation2d(p[0], p[1]) for p in path]
