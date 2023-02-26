from typing import Optional
from magicbot.state_machine import AutonomousStateMachine, state
from dataclasses import dataclass

from controllers.arm import Setpoint, Setpoints
from controllers.movement import Movement
from controllers.score_game_piece import ScoreGamePieceController
from controllers.acquire_cube import AcquireCubeController

from utilities.functions import grid_col_to_field_y

from wpimath.geometry import Pose2d, Rotation2d, Translation2d


from math import radians


@dataclass
class PickupPath:
    goal: Pose2d
    approach_direction: Rotation2d
    intermediate_waypoints: list[Translation2d]


@dataclass
class ScorePath:
    goal: Pose2d
    approach_direction: Rotation2d
    intermediate_waypoints: list[Translation2d]
    arm_setpoint: Setpoint


class AutoBase(AutonomousStateMachine):
    movement: Movement
    score_game_piece: ScoreGamePieceController
    acquire_cube: AcquireCubeController

    INTAKE_PRE_TIME = 2.0
    MANUAL_CUBE_TIME = 1.5

    def __init__(self) -> None:
        self.pickup_paths:list[PickupPath] = []
        self.score_paths:list[ScorePath] = []
        self.progress_idx = 0

    @state(first=True)
    def score(self, initial_call: bool) -> None:
        if initial_call:
            self.score_game_piece.engage()
        elif not self.score_game_piece.is_executing:
            self.next_state("pickup_cube")

    @state
    def approach_cube(self, initial_call: bool) -> None:
        if initial_call:
            path = self.pickup_paths[self.progress_idx]
            self.movement.set_goal(
                path.goal,
                path.approach_direction,
                waypoints=tuple(path.intermediate_waypoints),
            )
        self.movement.do_autodrive()
        if self.movement.time_to_goal < self.INTAKE_PRE_TIME:
            self.next_state("pickup_cube")

    @state
    def pickup_cube(self, initial_call, state_tm):
        if initial_call is True:
            self.movement.do_autodrive()
            self.acquire_cube.engage()
        elif not self.acquire_cube.is_executing:
            self.progress_idx += 1
            self.next_state("score")

        if state_tm > self.MANUAL_CUBE_TIME:
            self.acquire_cube.manual_cube_present()


class AutoTest(AutoBase):
    MODE_NAME = "Auto Test"
    DEFAULT = True

    def setup(self) -> None:
        self.score_paths = [
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(0), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                [],
                Setpoints.SCORE_CONE_HIGH,
            ),
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(1), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                [],
                Setpoints.SCORE_CUBE_HIGH,
            ),
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(4), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                [],
                Setpoints.SCORE_CUBE_HIGH,
            ),
        ]
        self.pickup_paths = [
            PickupPath(
                Pose2d(6.65885, 0.88812, Rotation2d(radians(4.56))),
                Rotation2d.fromDegrees(4.56),
                [],
            ),
            PickupPath(
                Pose2d(6.719, 1.92247, Rotation2d(radians(32.04))),
                Rotation2d.fromDegrees(32.04),
                [],
            ),
        ]
