from hypothesis import given
from hypothesis.strategies import floats
from utilities.pathfinding import Rect, intersect, find_path, to_trajectory
from wpimath.geometry import Translation2d

sensible_floats = floats(allow_infinity=False, allow_nan=False, width=16)


def test_line_line_intersect():
    assert intersect((0, 0), (1, 1), (1, 0), (0, 1)) is True
    assert intersect((0, 0), (2, 0), (0, 1), (0.5, 0.5)) is False


def test_line_rect_intersect():
    rect = Rect(1, 1, 3, 3)
    assert rect.line_intersect((2, 0), (2, 2)) is True
    assert rect.line_intersect((0, 2), (2, 2)) is True
    assert rect.line_intersect((4, 2), (2, 2)) is True
    assert rect.line_intersect((2, 4), (2, 2)) is True
    assert rect.line_intersect((2, 0), (2, 4)) is True
    assert rect.line_intersect((0, 2), (4, 2)) is True
    assert rect.line_intersect((2, 0), (2, 4)) is True

    assert rect.line_intersect((0, 0), (2, 0)) is False
    assert rect.line_intersect((0, 0), (0, 0)) is False
    assert rect.line_intersect((1.5, 1.5), (2.5, 2.5)) is True


sensible_floats = floats(allow_infinity=False, allow_nan=False, width=16)


@given(
    x1=sensible_floats,
    y1=sensible_floats,
    x2=sensible_floats,
    y2=sensible_floats,
)
def test_find_path(x1: float, y1: float, x2: float, y2: float):
    find_path(Translation2d(x1, y2), Translation2d(x2, y2))


def test_to_trajectory():
    to_trajectory([Translation2d(), Translation2d(1, 1)])
    to_trajectory([Translation2d()])
    to_trajectory([])
