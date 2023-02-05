from typing import Optional
from magicbot.state_machine import AutonomousStateMachine, state
from dataclasses import dataclass

from components.chassis import Chassis
from components.intake import Intake
from components.arm import Arm, Setpoints, Setpoint
from components.gripper import Gripper

from controllers.movement import Movement

from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from utilities.functions import grid_col_to_field_y

from math import radians


@dataclass
class PickupPath:
    goal: Pose2d
    approach_direction: Rotation2d
    intermediate_waypoints: Optional[list[Translation2d]]


@dataclass
class ScorePath:
    goal: Pose2d
    approach_direction: Rotation2d
    intermediate_waypoints: Optional[list[Translation2d]]
    arm_setpoint: Setpoint


class AutoBase(AutonomousStateMachine):
    gripper: Gripper
    arm: Arm
    chassis: Chassis
    intake: Intake
    movement: Movement

    def __init__(self) -> None:
        self.piece_positions = []
        self.pickup_paths = []
        self.score_paths = []
        self.stage = 0
        self.timeout_set = False
        self.timeout_start = 0

    @state(first=True)
    def score(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.timeout_set = False
            path = self.score_paths[self.stage]
            self.movement.next_state("autodrive")
            self.movement.set_goal(
                path.goal,
                path.approach_direction,
                waypoints=path.intermediate_waypoints,
            )
        self.movement.do_autodrive()
        if self.movement.time_remaining < 0.5:
            # self.arm.set_setpoint(self.score_paths[self.stage].arm_setpoint)
            pass
        if (
            self.movement.is_at_goal()
            # and self.arm.at_goal_angle()
            # and self.arm.at_goal_extension()
        ):
            self.gripper.open()
        if self.gripper.opened:
            # self.arm.set_setpoint(Setpoints.HANDOFF)
            if self.stage >= len(self.pickup_paths):
                self.done()
            self.next_state("pickup_cube")

    @state
    def pickup_cube(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.timeout_set = False
            path = self.pickup_paths[self.stage]
            self.movement.next_state("autodrive")
            self.movement.set_goal(
                path.goal,
                path.approach_direction,
                waypoints=path.intermediate_waypoints,
            )
        self.movement.do_autodrive()
        if self.movement.time_remaining < 0.5:
            if state_tm > 0.1 and not self.timeout_set:
                self.timeout_set = True
                self.timeout_start = Timer.getFPGATimestamp()
            self.intake.deploy()
        if (
            self.movement.is_at_goal()
            and self.intake.is_game_piece_present()
            or (
                self.timeout_set and Timer.getFPGATimestamp() - self.timeout_start > 2.0
            )
        ):
            self.intake.retract()
            self.gripper.close()
            if self.gripper.get_full_closed():
                self.stage += 1
                self.next_state("score")


class AutoTest(AutoBase):
    MODE_NAME = "Auto Test"
    DEFAULT = True

    def setup(self) -> None:
        self.score_paths = [
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(0), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                None,
                Setpoints.SCORE_CONE_HIGH,
            ),
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(1), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                None,
                Setpoints.SCORE_CUBE_HIGH,
            ),
            ScorePath(
                Pose2d(1.88795, grid_col_to_field_y(4), Rotation2d(0.0)),
                Rotation2d.fromDegrees(180.0),
                None,
                Setpoints.SCORE_CUBE_HIGH,
            ),
        ]
        self.pickup_paths = [
            PickupPath(
                Pose2d(6.65885, 0.88812, Rotation2d(radians(4.56))),
                Rotation2d.fromDegrees(4.56),
                None,
            ),
            PickupPath(
                Pose2d(6.719, 1.92247, Rotation2d(radians(32.04))),
                Rotation2d.fromDegrees(32.04),
                None,
            ),
        ]
