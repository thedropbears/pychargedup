from components.arm import Arm, MIN_EXTENSION, MAX_EXTENSION, MIN_ANGLE, MAX_ANGLE

import math

from utilities.game import Node, GamePiece, Rows

from magicbot import StateMachine, state, feedback

from utilities.functions import clamp


class Setpoint:
    # arm angle in radians
    # angle of 0 points towards positive X, at the intake
    # positive counterclockwise when looking at the left side of the robot
    angle: float
    # extension in meters, length from center of rotation to center of where pieces are held in the end effector
    extension: float

    def __eq__(self, other):
        if isinstance(other, Setpoint):
            return self.angle == other.angle and self.extension == other.extension
        return False

    def __str__(self):
        return f"({self.angle}, {self.extension})"

    def __init__(self, angle: float, extension: float) -> None:
        self.extension = clamp(extension, MIN_EXTENSION, MAX_EXTENSION)
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
    PREPARE_PICKUP_CONE = Setpoint(math.radians(-170), MIN_EXTENSION)
    PICKUP_CONE = Setpoint(math.radians(-170), MIN_EXTENSION + 0.15)
    HANDOFF = Setpoint(math.radians(45), MIN_EXTENSION)
    STOW = Setpoint(math.radians(-10), MIN_EXTENSION)
    START = Setpoint(math.radians(20), MIN_EXTENSION)
    SCORE_CONE_MID = Setpoint(math.radians(-160), MIN_EXTENSION)
    SCORE_CUBE_MID = Setpoint(math.radians(-180), MIN_EXTENSION)
    SCORE_CONE_HIGH = Setpoint(math.radians(-165), 1.17)
    SCORE_CUBE_HIGH = Setpoint(math.radians(-170), 1.17)

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
        self._target_setpoint: Setpoint = Setpoints.START
        self._about_to_run: bool = False

    def go_to_setpoint(self, setpoint: Setpoint) -> None:
        # Only restart the state machine if the setpoint is different
        if setpoint != self._target_setpoint:  # and self.at_goal():
            self._target_setpoint = setpoint
            self._about_to_run = True
            self.engage()

    @feedback
    def get_target(self) -> str:
        return str(self._target_setpoint)

    def get_angle(self) -> float:
        return self.arm_component.get_angle()

    def at_goal(self) -> bool:
        return not (self.is_executing or self._about_to_run)

    def is_at_forward_limit(self) -> bool:
        return self.arm_component.is_at_forward_limit()

    def is_at_retraction_limit(self) -> bool:
        return self.arm_component.is_retracted()

    @state(first=True, must_finish=True)
    def retracting_arm(self) -> None:
        self._about_to_run = False
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

    def on_enable(self) -> None:
        self.stop()
