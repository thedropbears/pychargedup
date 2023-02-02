from magicbot import (
    state,
    default_state,
    StateMachine,
    tunable,
    feedback
)

import math

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm
from controllers.movement import Movement

from enum import Enum, auto

class Piece(Enum):
    CUBE = auto()
    CONE = auto()
    NONE = auto()

# TODO: use set points

class ScoreController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    MOVE_FORWARD = 1
    # the distance that a cone will be at when we pick it up (meters)
    CONE_DISTANCE = 0.3

    has_piece: bool
    
    def __init__(self) -> None:
        self.has_piece = False
        self.holding_piece = Piece.NONE

    def setup(self) -> None:
        ...

    @state
    def pickup_cube_ground(self):
        """
        A function to pickup a cube from the ground
        Requires the robot to already be aligned to the cube
        """
        # turn the intake on
        self.intake.deploy()
        # move forward a bit to suck the piece in until the cube is not in reach (and therefore in our bot)
        if not(self.game_piece_in_reach()):
            current_goal = self.movement.goal
            self.movement.set_goal(Pose2d(current_goal.X(), current_goal.Y() + self.MOVE_FORWARD, current_goal.rotation()), current_goal.rotation())
        self.intake.retract()

        self.holding_piece = Piece.CUBE

    @state
    def pickup_cone(self):
        target_pos = self.arm.get_arm_from_target(self.CONE_DISTANCE, 0.95)
        # this means we can reach the position of ({CONE_DISTANCE}m, .95m)
        if target_pos[2]:
            # move the arm to target_pos
            self.arm.set_angle(target_pos[0])
            self.arm.set_length(target_pos[1])
            if self.arm.at_goal_angle() and self.arm.at_goal_extension():
                self.has_piece = True
                self.gripper.close()

                if self.get_time_to_goal() < 3:
                    self.arm.set_angle(math.radians(90))
                    self.arm.set_length(self.arm.MIN_EXTENSION)
                else:
                    self.arm.set_angle(math.radians(180))
                    self.arm.set_length(self.arm.MIN_EXTENSION)

            self.holding_piece = Piece.CONE
        else:
            raise Exception(f"The position ({self.CONE_DISTANCE}, .95) cannot be reached")
        
    @state
    def score_cone(self):
        ...
        
    def get_time_to_goal(self) -> float:
        ...

    @state
    def score_cube(self):
        ...

    @state(must_finish=True)
    def recovery(self):
        ...


    def gripper_open(self) -> None:
        self.has_piece = False # if the gripper is open, there is 100% no piece in our control
        self.gripper.open()

    @feedback
    def has_piece_grabbed(self) -> bool:
        return self.has_piece and self.gripper.get_full_closed()

    def gripper_close(self) -> None:
        self.gripper.close()
        if self.game_piece_in_reach() and self.gripper.get_full_closed():
            self.has_piece = True

    @feedback
    def game_piece_in_reach(self) -> bool:
        return self.gripper.game_piece_in_reach()

    @feedback
    def held_piece(self) -> Piece:
        return self.holding_piece

    