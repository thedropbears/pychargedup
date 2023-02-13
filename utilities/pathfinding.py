import math
from utilities.game import FIELD_LENGTH
from dataclasses import dataclass
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from typing import Optional
import astar  # type: ignore

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

    def point_intersect(self, p: Point) -> bool:
        return (
            p[0] >= self.x_min
            and p[0] < self.x_max
            and p[1] >= self.y_min
            and p[1] < self.y_max
        )

    def line_intersect(self, A: Point, B: Point) -> bool:
        # if self.point_intersect(A) or self.point_intersect(B):
        #     return True
        return (
            intersect(A, B, (self.x_min, self.y_min), (self.x_max, self.y_min))
            or intersect(A, B, (self.x_max, self.y_min), (self.x_max, self.y_max))
            or intersect(A, B, (self.x_max, self.y_max), (self.x_min, self.y_max))
            or intersect(A, B, (self.x_min, self.y_max), (self.x_min, self.y_min))
        )

    def flip(self) -> "Rect":
        return Rect(
            FIELD_LENGTH - self.x_max, self.y_min, FIELD_LENGTH - self.x_min, self.y_max
        )

    def expand(self, amount) -> None:
        self.x_min -= amount
        self.y_min -= amount
        self.x_max += amount
        self.y_max += amount


ROBOT_LEN = 1.0105
ROBOT_WIDTH = 0.8705

CHARGE_STATION = Rect(2.925, 1.527, 4.859, 3.947)
CHARGE_STATION.expand(0.4)
DIVIDER = Rect(0, 5.45, 3.36, 5.5)
DIVIDER.expand(0.4)

OBSTACLES = [CHARGE_STATION, CHARGE_STATION.flip(), DIVIDER, DIVIDER.flip()]


def is_visible(p1: Point, p2: Point) -> bool:
    return all(not ob.line_intersect(p1, p2) for ob in OBSTACLES)


waypoints_blue: list[Point] = [
    (2.600, 0.754),
    (5.600, 0.754),
    (2.600, 4.732),
    (5.600, 4.732),
    (4.000, 6.118),
]
waypoints_red: list[Point] = []
for w in waypoints_blue:
    waypoints_red.append((FIELD_LENGTH - w[0], w[1]))

waypoints = [*waypoints_blue, *waypoints_red]


def _find_path(start: Point, goal: Point) -> Optional[list[Point]]:
    new_waypoints = waypoints + [start, goal]
    iter_path = astar.find_path(
        start,
        goal,
        neighbors_fnct=lambda p: [x for x in new_waypoints if is_visible(x, p)],
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
        distance_between_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
    )
    if iter_path is None:
        return None
    return list(iter_path)


def find_path(start: Translation2d, end: Translation2d) -> list[Translation2d]:
    path = _find_path((start.x, start.y), (end.x, end.y))
    if path is None:
        print("no path")
        return [start, end]
    return [Translation2d(p[0], p[1]) for p in path]


def to_poses(path: list[Translation2d]) -> list[Pose2d]:
    return [Pose2d(t, Rotation2d()) for t in path]
