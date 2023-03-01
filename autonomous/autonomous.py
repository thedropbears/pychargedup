from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.geometry import Rotation2d, Translation2d
from dataclasses import dataclass
from components.arm import Arm

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

    INTAKE_PRE_TIME = 2.0
    SCORE_PRE_TIME = 2.0
    MANUAL_CUBE_TIME = 1.0

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
    def score_cone(self, initial_call: bool) -> None:
        if initial_call:
            self.arm_component.set_at_min_extension()
            self.recover.has_initialized_arm = True
            self.recover.done()
            self.score_game_piece.set_score_node(
                self.score_actions[self.progress_idx].node
            )
            self.score_game_piece.engage("deploying_arm")
        elif not self.score_game_piece.is_executing:
            self.next_state("approach_cube")

    @state
    def approach_cube(self, initial_call: bool) -> None:
        if initial_call:
            path = self.pickup_actions[self.progress_idx].with_correct_flipped()
            self.movement.set_goal(
                *get_staged_pickup(path.piece_idx, path.approach_direction),
                path.intermediate_waypoints,
                slow_dist=0
            )
        elif self.movement.time_to_goal < self.INTAKE_PRE_TIME:
            self.next_state("pickup_cube")
        self.movement.do_autodrive()

    @state
    def pickup_cube(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            self.acquire_cube.engage()
            self.recover.done()
        elif not self.acquire_cube.is_executing:
            self.next_state("approach_grid")
            self.progress_idx += 1
            return

        self.movement.do_autodrive()
        if state_tm - self.INTAKE_PRE_TIME > self.MANUAL_CUBE_TIME:
            self.acquire_cube.manual_cube_present()

    @state
    def approach_grid(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            path = self.score_actions[self.progress_idx].with_correct_flipped()
            self.movement.set_goal(
                *get_score_location(path.node), path.intermediate_waypoints
            )
        self.movement.do_autodrive()
        if self.movement.time_to_goal < self.SCORE_PRE_TIME:
            self.next_state("score_cube")

    @state
    def score_cube(self, initial_call: bool) -> None:
        if initial_call:
            self.recover.done()
            self.score_game_piece.engage("deploying_arm")
        elif not self.score_game_piece.is_executing:
            if self.progress_idx >= len(self.pickup_actions):
                self.done()
                return
            self.next_state("approach_cube")
        self.movement.do_autodrive()


class LoadingSide3(AutoBase):
    MODE_NAME = "Loading side 3 piece"
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
            ScoreAction(
                Node(Rows.MID, 7),
                (Translation2d(5.3, 4.4),),
            ),
        ]
        self.pickup_actions = [
            PickupAction(
                3,
                Rotation2d.fromDegrees(0),
                (),
            ),
            PickupAction(
                2,
                Rotation2d.fromDegrees(-20),
                (Translation2d(3, 4.9), Translation2d(5, 4.5)),
            ),
        ]
