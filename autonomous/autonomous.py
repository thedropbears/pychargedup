from magicbot.state_machine import AutonomousStateMachine, state
from dataclasses import dataclass

from components.chassis import Chassis
from components.intake import Intake
from components.arm import Arm, Setpoints, Setpoint
from components.gripper import Gripper

from controllers.movement import Movement

from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d


@dataclass
class PickupPath:
    goal: Pose2d
    approach_angle: Rotation2d
    intermediate_waypoints: list[Pose2d]


@dataclass
class ScorePath:
    goal: Pose2d
    approach_angle: Rotation2d
    intermediate_waypoints: list[Pose2d]
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
            self.movement.set_goal(
                path.goal,
                path.approach_direction,
                intermediate_waypoints=path.intermediate_waypoints,
            )
        self.movement.do_autodrive()
        if self.movement.time_remaining < 0.5:
            if not self.timeout_set:
                self.timeout_start = Timer.getFPGATimestamp()
                self.timeout_set = True
            self.arm.set_setpoint(self.score_paths[self.stage].arm_setpoint)
        if (
            self.movement.is_at_goal()
            and self.arm.at_goal_angle()
            and self.arm.at_goal_extension()
        ):
            self.gripper.open()
        if self.gripper.opened or Timer.getFPGATimestamp - self.timeout_start > 2.0:
            self.gripper.close()
            self.arm.set_setpoint(Setpoints.HANDOFF)
            self.next_state("pickup_cube")

    @state
    def pickup_cube(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.timeout_set = False
            path = self.pickup_paths[self.stage]
            self.movement.set_goal(
                path.goal,
                path.approach_direction,
                intermediate_waypoints=path.intermediate_waypoints,
            )
        self.movement.do_autodrive()
        if self.movement.time_remaining < 0.5:
            if not self.timeout_set:
                self.timeout_start = Timer.getFPGATimestamp()
                self.timeout_set = True
            self.intake.deploy()
        if (
            self.movement.is_at_goal() and self.intake.is_game_piece_present()
        ) or Timer.getFPGATimestamp - self.timeout_start > 1.0:
            self.intake.retract()
            self.stage += 1
            self.next_state("score")
