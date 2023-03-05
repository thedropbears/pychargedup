from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
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
    field_flip_pose2d,
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


@dataclass
class StationLocation:
    pose: Pose2d
    approach_angle: Rotation2d
    intermediate_waypoints: tuple[Translation2d, ...]

    def with_correct_fipped(self) -> "StationLocation":
        if is_red():
            waypoints = tuple(
                field_flip_translation2d(x) for x in self.intermediate_waypoints
            )
            return StationLocation(
                field_flip_pose2d(self.pose),
                field_flip_rotation2d(self.approach_angle),
                waypoints,
            )
        return self


class AutoBase(AutonomousStateMachine):
    """Alternates between the action in the score and pickup lists"""

    movement: Movement
    score_game_piece: ScoreGamePieceController
    recover: RecoverController
    arm_component: Arm
    gripper: Gripper

    INTAKE_PRE_TIME = 2.5
    SCORE_PRE_TIME = 2.5
    MANUAL_CUBE_TIME = 0.5

    def __init__(self) -> None:
        self.pickup_actions: list[PickupAction] = []
        self.score_actions: list[ScoreAction] = []
        self.station_location: StationLocation
        self.progress_idx = 0
        self.start_pose, _ = get_score_location(self.score_actions[0].node)

    def on_enable(self) -> None:
        self.start_pose, _ = get_score_location(self.score_actions[0].node)
        self.movement.chassis.set_pose(self.start_pose)
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
            self.next_state("approach_station")

    @state
    def approach_station(self, initial_call: bool) -> None:
        if initial_call:
            self.recover.done()
            self.movement.set_goal(
                self.station_location.pose,
                self.station_location.approach_angle,
                self.station_location.intermediate_waypoints,
            )
        elif self.movement.is_at_goal():
            self.next_state("balance")
        self.movement.do_autodrive()

    @state
    def balance(self, initial_call: bool, tm: float) -> None:
        if initial_call:
            self.movement.toggle_balance()
        elif not self.movement.is_executing:
            print(f"Finished auto at {tm}")
            self.done()
            return


class ScoreAndBalance(AutoBase):
    MODE_NAME = "Score cone and balance on station"
    DEFAULT = True

    def setup(self) -> None:
        self.score_actions = [
            ScoreAction(
                Node(Rows.HIGH, 8),
                (),
            ),
        ]
        self.station_location = StationLocation(
            Pose2d(
                self.start_pose.X() + 2, self.start_pose.Y(), self.start_pose.rotation()
            ),
            Rotation2d.fromDegrees(180),
            (),
        )
