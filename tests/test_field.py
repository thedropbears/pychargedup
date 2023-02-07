from hypothesis import given
from hypothesis.strategies import floats

from utilities.field import (
    field_flip_pose2d,
    field_flip_rotation2d,
    field_flip_translation2d,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

sensible_floats = floats(allow_infinity=False, allow_nan=False, width=16)


@given(x=sensible_floats, y=sensible_floats, angle=sensible_floats)
def test_flip_double(x: float, y: float, angle: float):
    p = Pose2d(x, y, angle)
    t = Translation2d(x, y)
    r = Rotation2d(angle)
    assert field_flip_translation2d(field_flip_translation2d(t)) == t
    assert field_flip_rotation2d(field_flip_rotation2d(r)) == r
    assert field_flip_pose2d(field_flip_pose2d(p)) == p
