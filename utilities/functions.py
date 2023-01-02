import math
import numpy as np
from typing import Tuple
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d


def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi,pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def rate_limit_2d(
    cur: Tuple[float, float], target: Tuple[float, float], rate_limit: float, dt: float
):
    """Limits the change in a vector to rate_limit * dt"""
    err = (target[0] - cur[0], target[1] - cur[1])
    mag = math.hypot(*err)
    if mag == 0:
        return target
    if mag < rate_limit * dt:
        change = err
    else:
        err_norm = (err[0] / mag, err[1] / mag)
        change = (err_norm[0] * rate_limit * dt, err_norm[1] * rate_limit * dt)
    return cur[0] + change[0], cur[1] + change[1]


def rate_limit_module(
    cur: SwerveModuleState,
    target: SwerveModuleState,
    rate_limit: float,
    dt: float = 0.02,
):
    """
    Limit the change in a module state so that the acceleration dosent exceed rate_limit
    """
    cur_vx = cur.angle.cos() * cur.speed
    cur_vy = cur.angle.sin() * cur.speed
    target_vx = target.angle.cos() * target.speed
    target_vy = target.angle.sin() * target.speed

    new_vx, new_vy = rate_limit_2d(
        (cur_vx, cur_vy), (target_vx, target_vy), rate_limit, dt
    )
    new_speed = math.hypot(new_vx, new_vy)
    if new_speed == 0:
        rot = target.angle
    else:
        rot = Rotation2d(new_vx, new_vy)
    return SwerveModuleState(new_speed, rot)


def clamp_2d(val: Tuple[float, float], radius: float):
    """
    Constrains a vector to be within the unit circle
    """
    mag = math.hypot(*val)
    if mag == 0:
        return (0, 0)
    new_mag = min(mag, radius)
    return new_mag * val[0] / mag, new_mag * val[1] / mag
