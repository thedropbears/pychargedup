from magicbot.state_machine import AutonomousStateMachine, state
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from dataclasses import dataclass
from components.arm import Arm
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController

from utilities.game import (
    Node,
    Rows,
    field_flip_rotation2d,
    field_flip_translation2d,
    field_flip_pose2d,
    get_score_location,
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
class FieldPose:
    pose: Pose2d
    approach_angle: Rotation2d
    intermediate_waypoints: tuple[Translation2d, ...]

    def with_correct_fipped(self) -> "FieldPose":
        if is_red():
            waypoints = tuple(
                field_flip_translation2d(x) for x in self.intermediate_waypoints
            )
            return FieldPose(
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
        self.station_location: list[FieldPose] = []
        self.progress_idx = 0
        self.balance_progress_index = 0

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
            self.next_state("move_to_station_y")

    @state
    def move_to_station_y(self, initial_call: bool) -> None:
        if initial_call:
            self.recover.done()
            path = self.station_location[
                self.balance_progress_index
            ].with_correct_fipped()
            self.movement.set_goal(
                path.pose,
                path.approach_angle,
                path.intermediate_waypoints,
            )
        elif self.movement.is_at_goal():
            self.balance_progress_index += 1
            self.next_state("move_out_of_community")
        self.movement.do_autodrive()

    @state
    def move_out_of_community(self, initial_call: bool) -> None:
        if initial_call:
            self.recover.done()
            path = self.station_location[
                self.balance_progress_index
            ].with_correct_fipped()
            self.movement.set_goal(
                path.pose,
                path.approach_angle,
                path.intermediate_waypoints,
            )
        elif self.movement.is_at_goal():
            self.balance_progress_index += 1
            self.next_state("approach_station")
        self.movement.do_autodrive()

    @state
    def approach_station(self, initial_call: bool) -> None:
        if initial_call:
            path = self.station_location[
                self.balance_progress_index
            ].with_correct_fipped()
            self.movement.set_goal(
                path.pose,
                path.approach_angle,
                path.intermediate_waypoints,
            )
        elif self.movement.is_at_goal():
            self.next_state("balance")
        self.movement.do_autodrive()

    @state
    def balance(self, initial_call: bool, tm: float) -> None:
        if initial_call:
            self.movement.toggle_balance()
        elif not self.movement.is_executing:
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

    def on_enable(self) -> None:
        start_pose, _ = get_score_location(self.score_actions[0].node)
        if is_red():
            start_pose = field_flip_pose2d(start_pose)
        self.station_location = [
            FieldPose(
                Pose2d(start_pose.X() + 0.25, 2.748, start_pose.rotation()),
                Rotation2d.fromDegrees(270),
                (),
            ),
            FieldPose(
                Pose2d(start_pose.X() + 3.2, 2.748, start_pose.rotation()),
                Rotation2d.fromDegrees(0) if is_red() else Rotation2d.fromDegrees(180),
                (),
            ),
            FieldPose(
                Pose2d(start_pose.X() + 2.5, 2.748, start_pose.rotation()),
                Rotation2d.fromDegrees(180) if is_red() else Rotation2d.fromDegrees(0),
                (),
            ),
        ]
        print(start_pose, self.station_location[0].pose)
        return super().on_enable()
