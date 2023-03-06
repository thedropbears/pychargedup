from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.geometry import Rotation2d, Translation2d
from dataclasses import dataclass
from components.arm import Arm
from components.gripper import Gripper
from components.intake import Intake

from controllers.movement import Movement
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController
from controllers.acquire_cube import AcquireCubeController

from utilities.game import (
    Node,
    Rows,
    field_flip_rotation2d,
    field_flip_translation2d,
    get_score_location,
    get_staged_pickup,
    is_red,
)


@dataclass
class PickupAction:
    piece_idx: int
    approach_direction: Rotation2d
    intermediate_waypoints: tuple[Translation2d, ...]

    def with_correct_flipped(self) -> "PickupAction":
        if is_red():
            waypoints = tuple(
                field_flip_translation2d(x) for x in self.intermediate_waypoints
            )
            return PickupAction(
                self.piece_idx,
                field_flip_rotation2d(self.approach_direction),
                waypoints,
            )
        return self


@dataclass
class ScoreAction:
    node: Node
    intermediate_waypoints: tuple[Translation2d, ...]

    def with_correct_flipped(self) -> "ScoreAction":
        if is_red():
            waypoints = tuple(
                field_flip_translation2d(x) for x in self.intermediate_waypoints
            )
            return ScoreAction(self.node, waypoints)
        return self


class AutoBase(AutonomousStateMachine):
    """Alternates between the action in the score and pickup lists"""

    movement: Movement
    score_game_piece: ScoreGamePieceController
    acquire_cube: AcquireCubeController
    recover: RecoverController
    arm_component: Arm
    gripper: Gripper
    intake: Intake

    INTAKE_PRE_TIME = 2.5
    SCORE_PRE_TIME = 2.5
    MANUAL_CUBE_TIME = 0.5

    MAX_VEL = 2.0
    MAX_ACCEl = 2.0

    def __init__(self) -> None:
        self.pickup_actions: list[PickupAction] = []
        self.score_actions: list[ScoreAction] = []
        self.progress_idx = 0

    def on_enable(self) -> None:
        start_pose, _ = get_score_location(self.score_actions[0].node)
        self.movement.chassis.set_pose(start_pose)
        self.progress_idx = 0
        return super().on_enable()

    @state(first=True)
    def initialise(self, initial_call):
        if initial_call:
            self.gripper.close()
            self.recover.engage()
        elif not self.recover.is_executing:
            self.next_state("score_cone")

    @state
    def score_cone(self, initial_call: bool) -> None:
        if initial_call:
            self.score_game_piece.score_without_moving(
                self.score_actions[self.progress_idx].node
            )
        elif not self.score_game_piece.is_executing:
            self.next_state("approach_cube")

    @state
    def approach_cube(self, initial_call: bool) -> None:
        if initial_call:
            action = self.pickup_actions[self.progress_idx].with_correct_flipped()
            self.movement.set_goal(
                *get_staged_pickup(action.piece_idx, action.approach_direction),
                action.intermediate_waypoints,
                slow_dist=0,
                max_accel=self.MAX_ACCEl,
                max_vel=self.MAX_VEL,
            )
        elif self.movement.time_to_goal < self.INTAKE_PRE_TIME:
            self.next_state("pickup_cube")
        self.movement.do_autodrive()

    @state
    def pickup_cube(self, initial_call: bool, state_tm: float, tm: float) -> None:
        if initial_call:
            self.acquire_cube.engage()
            self.recover.done()
        elif not self.acquire_cube.is_executing:
            self.next_state("approach_grid")
            self.progress_idx += 1
            if self.progress_idx >= len(self.score_actions):
                print(f"Finished auto at {tm}")
                self.done()
                return

        self.movement.do_autodrive()
        if state_tm - self.INTAKE_PRE_TIME > self.MANUAL_CUBE_TIME:
            self.acquire_cube.override_cube_present = True

    @state
    def approach_grid(self, initial_call: bool) -> None:
        if initial_call:
            path = self.score_actions[self.progress_idx].with_correct_flipped()
            self.movement.set_goal(
                *get_score_location(path.node),
                path.intermediate_waypoints,
                max_accel=self.MAX_ACCEl,
                max_vel=self.MAX_VEL,
            )
        self.movement.do_autodrive()
        if self.movement.time_to_goal < self.SCORE_PRE_TIME:
            self.next_state("score_cube")

    @state
    def score_cube(self, initial_call: bool, tm: float) -> None:
        if initial_call:
            self.recover.done()
            self.score_game_piece.score_without_moving(
                self.score_actions[self.progress_idx].node
            )
        elif not self.score_game_piece.is_executing:
            if self.progress_idx >= len(self.pickup_actions):
                print(f"Finished auto at {tm}")
                self.done()
                return
            self.next_state("approach_cube")
        self.movement.do_autodrive()


class LoadingSide2(AutoBase):
    MODE_NAME = "Loading side 2 piece"
    DEFAULT = True

    def setup(self) -> None:
        self.score_actions = [
            ScoreAction(
                Node(Rows.HIGH, 8),
                (),
            ),
            ScoreAction(
                Node(Rows.HIGH, 7),
                (Translation2d(3.5, 4.7),),
            ),
        ]
        self.pickup_actions = [
            PickupAction(
                3,
                Rotation2d.fromDegrees(0),
                (),
            ),
        ]


class BumpSide2(AutoBase):
    MODE_NAME = "Bump side 2 piece"

    def setup(self) -> None:
        self.score_actions = [
            ScoreAction(
                Node(Rows.HIGH, 0),
                (),
            ),
            ScoreAction(
                Node(Rows.HIGH, 1),
                (Translation2d(3.5, 0.7),),
            ),
        ]
        self.pickup_actions = [
            PickupAction(
                0,
                Rotation2d.fromDegrees(0),
                (),
            ),
        ]
