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

    SCORE_PRE_TIME = 2.5

    MAX_VEL = 1.0
    MAX_ACCEl = 2.0

    def __init__(
        self, score_action: ScoreAction, leave_pose: FieldPose, balance_pose: FieldPose
    ) -> None:
        self.score_action = score_action
        self.leave_pose = leave_pose
        self.balance_pose = balance_pose

    def on_enable(self) -> None:
        start_pose, _ = get_score_location(self.score_action.node)
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
            self.score_game_piece.score_without_moving(self.score_action.node)
        elif not self.score_game_piece.is_executing:
            self.next_state("leave_community")

    @state
    def leave_community(self, initial_call: bool) -> None:
        if initial_call:
            self.recover.done()
            path = self.leave_pose.with_correct_fipped()
            self.movement.set_goal(
                path.pose,
                path.approach_angle,
                path.intermediate_waypoints,
                max_vel=self.MAX_VEL,
                max_accel=self.MAX_ACCEl,
            )
        elif self.movement.is_at_goal():
            self.next_state("approach_station")
        self.movement.do_autodrive()

    @state
    def approach_station(self, initial_call: bool) -> None:
        if initial_call:
            path = self.balance_pose.with_correct_fipped()
            self.movement.set_goal(
                path.pose,
                path.approach_angle,
                path.intermediate_waypoints,
                max_vel=self.MAX_VEL,
                max_accel=self.MAX_ACCEl,
            )
        elif self.movement.is_at_goal():
            self.next_state("balance")
        self.movement.do_autodrive()

    @state
    def balance(self, initial_call: bool, tm: float) -> None:
        if initial_call:
            self.movement.start_balance()
        elif not self.movement.is_executing or tm > 14.5:
            self.next_state("lock_wheels")

    @state
    def lock_wheels(self) -> None:
        self.movement.chassis.lock_swerve()


class ScoreAndBalance(AutoBase):
    MODE_NAME = "Score cone and balance on station"
    DEFAULT = True

    def __init__(self) -> None:
        super().__init__(
            ScoreAction(Node(Rows.HIGH, 3), ()),
            FieldPose(
                Pose2d(6.3, 2.3, Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                (),
            ),
            FieldPose(
                Pose2d(5.0, 2.3, Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(180),
                (),
            ),
        )