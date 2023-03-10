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
    PREPARE_PICKUP_CONE = Setpoint(math.radians(-183), MIN_EXTENSION)
    PICKUP_CONE = Setpoint(math.radians(-183), MIN_EXTENSION + 0.15)
    HANDOFF = Setpoint(math.radians(45), 0.93)
    STOW = Setpoint(math.radians(25), MIN_EXTENSION)
    SCORE_CONE_MID = Setpoint(math.radians(-180), MIN_EXTENSION)
    SCORE_CUBE_MID = Setpoint(math.radians(-180), MIN_EXTENSION)
    SCORE_CONE_HIGH = Setpoint(math.radians(-160), 1.18)
    SCORE_CUBE_HIGH = Setpoint(math.radians(-160), 1.18)

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
        self._about_to_run: bool = False

    def go_to_setpoint(self, setpoint: Setpoint, do_retract: bool = True) -> None:
        # If this is a different setpoint, we want to take it
        # Interrupt what we are doing and move to the new setpoint
        setpoint_changed = setpoint != self._target_setpoint
        # If it is the same setpoint we should check to see if we are actually there
        # Maybe we got interrupted or just started
        done_and_not_at_goal = not self.is_executing and not self.at_goal()

        if setpoint_changed or done_and_not_at_goal:
            self._about_to_run = True
            self._target_setpoint = setpoint
            if do_retract:
                self.engage("retracting_arm", force=True)
            else:
                self.engage("rotating_arm", force=True)

    def go_to_setpoint_without_retracting(self, setpoint: Setpoint) -> None:
        self.go_to_setpoint(setpoint, False)

    @feedback
    def get_target(self) -> str:
        return str(self._target_setpoint)

    def get_angle(self) -> float:
        return self.arm_component.get_angle()

    @feedback
    def at_goal(self) -> bool:
        return self.arm_component.at_pose(
            self._target_setpoint.angle, self._target_setpoint.extension
        )

    @property
    def is_executing(self):
        return super().is_executing or self._about_to_run

    def is_at_forward_limit(self) -> bool:
        return self.arm_component.get_wall_switch()

    def is_at_retraction_limit(self) -> bool:
        return self.arm_component.is_retracted()

    @state(first=True, must_finish=True)
    def retracting_arm(self) -> None:
        self._about_to_run = False
        if self.arm_component.use_voltage:
            self.arm_component.set_voltage(-2.0)
        self.arm_component.set_length(MIN_EXTENSION)
        if self.arm_component.at_goal_extension():
            self.next_state("rotating_arm")

    @state(must_finish=True)
    def rotating_arm(self, initial_call) -> None:
        self._about_to_run = False
        self.arm_component.set_angle(self._target_setpoint.angle)
        if self.arm_component.at_goal_angle() and not initial_call:
            self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        self._about_to_run = False
        self.arm_component.set_length(self._target_setpoint.extension)
        if self.arm_component.at_goal():
            self.done()

    def stop(self):
        self.arm_component.stop()
        super().done()

    def on_enable(self) -> None:
        self.stop()
