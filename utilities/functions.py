import math
import numpy as np
from typing import Tuple
from wpimath.kinematics import ChassisSpeeds


def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi,pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def rate_limit_2d(
    cur: ChassisSpeeds, target: ChassisSpeeds, rate_limit: float, dt: float = 0.02
):
    """
    Limit the change in chassis speeds so that the translation dosent exceed rate limit acceleration
    """
    err = (target.vx - cur.vx, target.vy - cur.vy)
    mag = math.hypot(*err)
    if mag == 0:
        return target
    if mag < rate_limit * dt:
        change = err
    else:
        err_norm = (err[0] / mag, err[1] / mag)
        change = (err_norm[0] * rate_limit * dt, err_norm[1] * rate_limit * dt)
    return ChassisSpeeds(cur.vx + change[0], cur.vy + change[1], target.omega)


def clamp_2d(val: Tuple[float, float], radius: float):
    """
    Constrains a vector to be within the unit circle
    """
    mag = math.hypot(*val)
    if mag == 0:
        return (0, 0)
    new_mag = min(mag, radius)
    return new_mag * val[0] / mag, new_mag * val[1] / mag
