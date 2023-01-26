import wpilib
import numpy as np
from enum import Enum, auto
from controllers.movement import Movement
from wpimath.geometry import Pose2d, Rotation2d
from components.chassis import Chassis

def slippage(
    value: int,
    slippage: int,
    wanted: int
) -> bool:
    (min, max) = (value * (1 - (slippage / 100)), value * (1 + (slippage / 100)))
    return min < wanted < max

class ChargeStationState(Enum):
    OFF    = auto()
    MIDWAY = auto()
    ON     = auto()

class ChargeStation:
    gyro: wpilib.interfaces.Gyro
    movement: Movement

    GYRO_CENTER = 1
    GYRO_OFFSET = 1.

    TILTED_STATION_DEGREES = 11

    ADJUSTMENT_METERS = 0.1

    def __init__(self, movement: Movement, chassis: Chassis):
        self.gyro = chassis.imu
        self.state = ChargeStationState.OFF
        self.movement = movement

    def setup(self) -> None:
        self.gyro.initGyro()
        self.gyro.calibrate()
        # self.gyro.reset()

    def get_angle(self) -> float:
        return self.gyro.getAngle()
    
    def set_state(self, state: ChargeStationState) -> None:
        self.state = state

    def move_off(self) -> None:
        # assuming we are already aligned with the station
        current_goal: Pose2d = self.movement.goal
        (x, y) = (current_goal.X(), current_goal.Y())
        (new_x, new_y) = (x, y + self.ADJUSTMENT_METERS)
        # check if we are now on the charge station
        if slippage(
            self.get_angle(),
            2, # 2% slippage, so if we are 2% of 11deg it still counts as being on the dashboard
            11
        ):
            self.state = ChargeStationState.MIDWAY
        else:
            self.movement.set_goal(Pose2d(new_x, new_y, current_goal.rotation()))

    def move_midway(self) -> None:
        current_goal: Pose2d = self.movement.goal
        (x, y) = (current_goal.X(), current_goal.Y())
        (new_x, new_y) = (x, y + self.ADJUSTMENT_METERS)
        # check if the charge station is level
        if slippage(
            self.get_angle(),
            2, # 2% slippage, so if we are 2% of 0deg it still counts as being on the dashboard
            0
        ):
            self.state = ChargeStationState.ON
        else:
            self.movement.set_goal(Pose2d(new_x, new_y, current_goal.rotation()))

    def move_on(self) -> None:
        return # nothing to do here

    def execute(self) -> None:
        if self.state == ChargeStationState.OFF:
            self.move_off()
        elif self.state == ChargeStationState.MIDWAY:
            self.move_midway()
        elif self.state == ChargeStationState.ON:
            self.move_on()

        return