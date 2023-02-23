from components.arm import Arm, MIN_EXTENSION, MAX_EXTENSION, MIN_ANGLE, MAX_ANGLE

import math

from utilities.game import Node, GamePiece, Rows

from magicbot import StateMachine, state

from utilities.functions import clamp


class Setpoint:
    # arm angle in radians
    # angle of 0 points towards positive X, at the intake
    # positive counterclockwise when looking at the left side of the robot
    angle: float
    # extension in meters, length from center of rotation to center of where pieces are held in the end effector
    extension: float

    def __init__(self, angle: float, extension: float) -> None:
        self.extension = clamp(extension, MIN_EXTENSION, MAX_EXTENSION)
        if angle > MAX_ANGLE:  # and (self.angle - math.tau) < Arm.MIN_ANGLE:
            angle -= math.tau
        self.angle = clamp(angle, MIN_ANGLE, MAX_ANGLE)

        if self.angle != angle or self.extension != extension:
            print(
                "SETPOINT WAS CLAMPED", (angle, extension), (self.angle, self.extension)
            )

    @classmethod
    def fromCartesian(cls, x: float, z: float) -> "Setpoint":
        """Sets a cartesian goal reletive to the arms axle"""
        return cls(math.atan2(-z, x), math.hypot(x, z))

    def toCartesian(self) -> tuple[float, float]:
        return self.extension * math.cos(self.angle), self.extension * math.sin(
            self.angle
        )


class Setpoints:
    PREPARE_PICKUP_CONE = Setpoint(-3.05, MIN_EXTENSION)
    PICKUP_CONE = Setpoint(-3.05, MAX_EXTENSION)
    HANDOFF = Setpoint(0.8, MIN_EXTENSION)
    STOW = Setpoint(math.radians(50), MIN_EXTENSION)
    START = Setpoint(-math.radians(30), MIN_EXTENSION)
    SCORE_CONE_MID = Setpoint(-2.8, MIN_EXTENSION)
    SCORE_CUBE_MID = Setpoint(-3.2, MIN_EXTENSION)
    SCORE_CONE_HIGH = Setpoint(-2.89, 1.17)
    SCORE_CUBE_HIGH = Setpoint(-3, 1.17)

    UPRIGHT = Setpoint(-math.pi / 2, MIN_EXTENSION + 0.1)
    FORWARDS = Setpoint(0, MIN_EXTENSION)
    BACKWARDS = Setpoint(-math.pi, MIN_EXTENSION)


def get_setpoint_from_node(node: Node) -> Setpoint:
    if node.row is Rows.LOW:
        return Setpoints.STOW
    elif node.row is Rows.MID:
        if node.get_valid_piece() is GamePiece.CONE:
            return Setpoints.SCORE_CONE_MID
        else:
            return Setpoints.SCORE_CONE_MID
    else:  # high
        if node.get_valid_piece() is GamePiece.CONE:
            return Setpoints.SCORE_CONE_HIGH
        else:
            return Setpoints.SCORE_CUBE_HIGH


class ArmController(StateMachine):
    arm_component: Arm

    def __init__(self) -> None:
        self._target_setpoint: Setpoint = Setpoints.STOW

    def go_to_setpoint(self, setpoint: Setpoint) -> None:
        # Only restart the state machine if the setpoint is different
        if setpoint != self._target_setpoint:
            self.done()
            self._target_setpoint = setpoint
            self.engage()

    def get_angle(self) -> float:
        return self.arm_component.get_angle()

    def at_goal(self) -> bool:
        return not self.is_executing

    @state(first=True, must_finish=True)
    def retracting_arm(self) -> None:
        self.arm_component.set_length(MIN_EXTENSION)
        if self.arm_component.at_goal_extension():
            self.next_state("rotating_arm")

    @state(must_finish=True)
    def rotating_arm(self) -> None:
        self.arm_component.set_angle(self._target_setpoint.angle)
        if self.arm_component.at_goal_angle():
            self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        self.arm_component.set_length(self._target_setpoint.extension)
        if self.arm_component.at_goal():
            self.done()

    def stop(self):
        self.arm_component.stop()
        super().done()
