from math import sin, cos, hypot
from pytest import approx
from hypothesis import given
from hypothesis.strategies import floats

from utilities.functions import clamp_2d, rate_limit_2d
from wpimath.kinematics import ChassisSpeeds


# chassis speeds dosent have an equals operator
def speeds_equals(a: ChassisSpeeds, b: ChassisSpeeds):
    return a.vx == b.vx and a.vy == b.vy and a.omega == b.omega


@given(
    vx=floats(allow_infinity=False, allow_nan=False),
    vy=floats(allow_infinity=False, allow_nan=False),
    vz=floats(allow_infinity=False, allow_nan=False),
)
def test_rate_limit_still(vx, vy, vz):
    speeds = ChassisSpeeds(vx, vy, vz)
    res = rate_limit_2d(speeds, speeds, 1, 1)
    assert speeds_equals(res, speeds)


@given(mag=floats(0, 1), angle=floats(allow_infinity=False, allow_nan=False))
def test_clamp2d_noconstrain(mag, angle):
    # generate a point in the unit circle
    x = cos(angle) * mag
    y = sin(angle) * mag
    # make sure its not contrained
    assert clamp_2d((x, y), 1) == approx((x, y))


@given(
    x=floats(allow_infinity=False, allow_nan=False),
    y=floats(allow_infinity=False, allow_nan=False),
)
def test_clamp2d_constrain(x, y):
    result = clamp_2d((x, y), 1)
    magnitude = hypot(*result)
    assert magnitude <= 1 or magnitude == approx(1)
