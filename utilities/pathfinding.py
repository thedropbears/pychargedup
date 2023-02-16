import math
from utilities.game import FIELD_LENGTH, FIELD_WIDTH, field_flip_pose2d
from dataclasses import dataclass
from wpimath.geometry import Pose2d
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
        if self.point_intersect(A) or self.point_intersect(B):
            return True
        c1 = (self.x_min, self.y_min)
        c2 = (self.x_max, self.y_min)
        c3 = (self.x_max, self.y_max)
        c4 = (self.x_min, self.y_max)
        return (
            intersect(A, B, c1, c2)
            or intersect(A, B, c2, c3)
            or intersect(A, B, c3, c4)
            or intersect(A, B, c4, c1)
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

    def get_corners(self) -> list[Pose2d]:
        c1 = Pose2d(self.x_min, self.y_min, 0)
        c2 = Pose2d(self.x_max, self.y_min, 0)
        c3 = Pose2d(self.x_max, self.y_max, 0)
        c4 = Pose2d(self.x_min, self.y_max, 0)
        return [c1, c2, c3, c4]

    def __eq__(self, __o: "Rect") -> bool:
        return (
            self.x_min == __o.x_min
            and self.x_max == __o.x_max
            and self.y_min == __o.y_min
            and self.y_max == __o.y_max
        )


OBSTICLE_BUFFER = 0.4

CHARGE_STATION = Rect(2.925, 1.527, 4.859, 3.947)
CHARGE_STATION.expand(OBSTICLE_BUFFER)
DIVIDER = Rect(0, 5.45, 3.36, 5.5)
DIVIDER.expand(OBSTICLE_BUFFER)
EDGES = [
    Rect(0, -0.5, FIELD_LENGTH, 0),
    Rect(-0.5, 0, 0, FIELD_WIDTH),
    Rect(0, FIELD_WIDTH, FIELD_LENGTH, FIELD_WIDTH + 0.5),
    Rect(FIELD_LENGTH, 0, FIELD_LENGTH + 0.5, FIELD_WIDTH),
]
for e in EDGES:
    e.expand(OBSTICLE_BUFFER)

OBSTACLES = [CHARGE_STATION, CHARGE_STATION.flip(), DIVIDER, DIVIDER.flip()]


def is_visible(p1: Point, p2: Point) -> bool:
    return all(not ob.line_intersect(p1, p2) for ob in OBSTACLES)


# x, y, rotation
waypoints_blue: list[Pose2d] = [
    Pose2d(2.300, 0.7, 0),
    Pose2d(5.600, 0.75, 0),
    Pose2d(2.400, 4.40, 0),
    Pose2d(5.700, 4.5, 0),
    Pose2d(4.000, 6.118, 0),
]
waypoints_red: list[Pose2d] = []
for w in waypoints_blue:
    waypoints_red.append(field_flip_pose2d(w))

all_waypoints = [*waypoints_blue, *waypoints_red]


def find_path(start: Pose2d, goal: Pose2d) -> list[Pose2d]:
    new_waypoints = all_waypoints + [start, goal]
    # cant use pose2d in find_path because it must be hashable
    tuples: list[tuple[float, float, float]] = [
        (p.x, p.y, p.rotation().radians()) for p in new_waypoints
    ]
    iter_path = astar.find_path(
        (start.x, start.y, start.rotation().radians()),
        (goal.x, goal.y, goal.rotation().radians()),
        neighbors_fnct=lambda a: [b for b in tuples if is_visible(b[:2], a)],
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
        distance_between_fnct=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
    )
    if iter_path is None:
        print("no path")
        return [start, goal]
    return [Pose2d(p[0], p[1], p[2]) for p in iter_path]


def get_all_corners() -> list[Pose2d]:
    corners = []
    for o in OBSTACLES:
        corners.extend(o.get_corners())
    return corners
