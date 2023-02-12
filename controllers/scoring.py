from magicbot import state, StateMachine, tunable, feedback
from wpimath.geometry import Pose2d, Rotation2d, Translation3d
import wpilib

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm, Setpoints, Setpoint
from controllers.movement import Movement
from utilities.game import (
    GamePiece,
    get_double_substation,
    get_single_substation,
    field_flip_rotation2d,
    RED_NODES,
    BLUE_NODES,
    Rows,
)


class ScoringController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    # how long before reaching pickup shelf to start closing claw
    GRAB_PRE_TIME = tunable(0.5)
    SCORE_PRE_TIME = tunable(0.1)
    # amont of time before arriving at goal to stow arm before which
    AUTO_STOW_CUTOFF = tunable(5)
    # how long before arriving at the correct position for each action
    # to move the arm to correct position or start intaking
    PICKUP_CUBE_PREPARE_TIME = tunable(3)
    SCORE_PREPARE_TIME = tunable(3)
    PICKUP_CONE_PREPARE_TIME = tunable(3)
    DESIRED_ROW = tunable(0)
    DESIRED_COLUMN = tunable(0)

    def __init__(self) -> None:
        self.is_holding = GamePiece.NONE
        self.autodrive = False
        self.wants_piece = GamePiece.CONE
        self.wants_to_intake = False
        # use double substation shelf on right side from drivers pov
        self.cone_pickup_side_right = False
        self.target_nodes = RED_NODES if self.is_red() else BLUE_NODES

    def get_correct_autodrive_state(self) -> str:
        cur_piece = self.get_current_piece()
        if cur_piece is GamePiece.CONE or cur_piece is GamePiece.CUBE:
            return "score"
        elif self.wants_piece is GamePiece.CONE:
            return "auto_pickup_cone"
        elif self.wants_piece is GamePiece.CUBE:
            return "auto_pickup_cube"
        return "idle"

    def goto_autodrive_state(self):
        self.next_state(self.get_correct_autodrive_state())

    @state(first=True)
    def idle(self):
        if self.autodrive:
            self.goto_autodrive_state()
        if self.wants_to_intake:
            self.next_state("intaking")

    @state
    def intaking(self):
        if self.autodrive:
            self.goto_autodrive_state()
        if not self.wants_to_intake:
            self.next_state("idle")

        self.arm.goto_setpoint(Setpoints.HANDOFF)
        self.intake.deploy()
        if self.intake.is_game_piece_present():
            self.intake.retract()
            self.next_state("grab_from_well")

    @state
    def grab_from_well(self):
        """Grabs a cube from the well, assumes there is a cube in the well"""
        self.arm.goto_setpoint(Setpoints.HANDOFF)
        self.gripper.open()
        if self.arm.at_goal():
            self.gripper.close()
            if self.gripper.get_full_closed():
                self.is_holding = GamePiece.CUBE
                self.next_state("idle")
                self.arm.goto_setpoint(Setpoints.STOW)

    @state
    def auto_pickup_cube(self):
        """Drives to intake a cube from the ground"""
        if not self.autodrive:
            self.next_state("idle")

        self.movement.set_goal(*self.get_cube_pickup())
        if self.movement.time_to_goal < self.PICKUP_CUBE_PREPARE_TIME:
            self.arm.goto_setpoint(Setpoints.HANDOFF)
            self.gripper.open()
            self.intake.deploy()
            if (
                self.intake.is_game_piece_present()
                or self.gripper.game_piece_in_reach()
            ):
                self.next_state("grab_from_well")
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.goto_setpoint(Setpoints.STOW)
        self.movement.do_autodrive()

    @state
    def auto_pickup_cone(self):
        if not self.autodrive:
            self.next_state("idle")

        self.movement.set_goal(*self.get_cone_pickup())
        self.intake.retract()
        if self.movement.time_to_goal < self.GRAB_PRE_TIME:  # or gripper sees cone
            self.gripper.close()
            if self.gripper.get_full_closed():
                self.is_holding = GamePiece.CONE
                self.next_state("idle")
                self.arm.goto_setpoint(Setpoints.STOW)
        elif self.movement.time_to_goal < self.PICKUP_CONE_PREPARE_TIME:
            self.arm.goto_setpoint(Setpoints.PICKUP_CONE)
            self.gripper.open()
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.goto_setpoint(Setpoints.STOW)

        self.movement.do_autodrive()

    @state
    def score(self, initial_call):
        if not self.autodrive:
            self.next_state("idle")

        if initial_call:
            move_goal, self.arm_setpoint = self.get_score_location()
            self.movement.set_goal(*move_goal)

        self.intake.retract()
        if self.movement.time_to_goal < self.SCORE_PRE_TIME:
            self.gripper.open()
            self.is_holding = GamePiece.NONE
            if self.gripper.get_full_open():
                self.next_state("idle")
                self.arm.goto_setpoint(Setpoints.STOW)
        elif self.movement.time_to_goal < self.SCORE_PREPARE_TIME:
            self.arm.goto_setpoint(self.arm_setpoint)
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.goto_setpoint(Setpoints.STOW)

        self.movement.do_autodrive()

    @feedback
    def is_red(self) -> bool:
        return self.get_team() == wpilib.DriverStation.Alliance.kRed

    def get_team(self) -> wpilib.DriverStation.Alliance:
        return wpilib.DriverStation.getAlliance()

    def get_current_piece(self) -> GamePiece:
        """What piece the gripper is currently holding in the gripper"""
        if self.gripper.get_full_closed() or self.gripper.is_opening():
            return self.is_holding
        return GamePiece.NONE

    @feedback
    def get_current_piece_str(self) -> str:
        return self.get_current_piece().name

    @feedback
    def get_wants_piece_str(self) -> str:
        return self.wants_piece.name

    def get_cube_pickup(self) -> tuple[Pose2d, Rotation2d]:
        """Gets where to auto pickup cubes from"""
        # can be changed for autonomous period?
        goal_trans = get_single_substation(wpilib.DriverStation.Alliance.kBlue)
        goal_rotation = (
            field_flip_rotation2d(Rotation2d(0)) if self.is_red() else Rotation2d(0)
        )
        return Pose2d(goal_trans, goal_rotation), goal_rotation

    def get_cone_pickup(self) -> tuple[Pose2d, Rotation2d]:
        is_wall_side = self.cone_pickup_side_right == self.is_red()
        goal_trans = get_double_substation(self.get_team(), is_wall_side)

        goal_rotation = (
            field_flip_rotation2d(Rotation2d(0)) if self.is_red() else Rotation2d(0)
        )
        offset_x, _ = Setpoints.PICKUP_CONE.toCartesian()

        # offset goal_trans to be where robot centre will be
        if self.is_red():
            goal_trans = goal_trans + Translation3d(-offset_x, 0, 0)
        else:
            goal_trans = goal_trans + Translation3d(offset_x, 0, 0)
        return (
            Pose2d(
                goal_trans.toTranslation2d(),
                goal_rotation + Rotation2d.fromDegrees(180),
            ),
            goal_rotation,
        )

    def get_score_location(self) -> tuple[tuple[Pose2d, Rotation2d], Setpoint]:
        # TODO: pick actual scoring node
        goal_trans = self.target_nodes[int(self.DESIRED_ROW)][int(self.DESIRED_COLUMN)]

        if self.get_current_piece() == GamePiece.CONE:
            if self.DESIRED_ROW == Rows.HIGH:
                offset_x, _ = Setpoints.SCORE_CONE_HIGH.toCartesian()
            elif self.DESIRED_ROW == Rows.MID:
                offset_x, _ = Setpoints.SCORE_CONE_MID.toCartesian()
        elif self.get_current_piece() == GamePiece.CUBE:
            if self.DESIRED_ROW == Rows.HIGH:
                offset_x, _ = Setpoints.SCORE_CUBE_HIGH.toCartesian()
            elif self.DESIRED_ROW == Rows.MID:
                offset_x, _ = Setpoints.SCORE_CUBE_MID.toCartesian()

        if self.is_red():
            goal_trans = goal_trans + Translation3d(+offset_x, 0, 0)
        else:
            goal_trans = goal_trans + Translation3d(-offset_x, 0, 0)

        goal_rot = Rotation2d.fromDegrees(180)
        goal_approach = Rotation2d.fromDegrees(0)
        return (
            Pose2d(goal_trans.toTranslation2d(), goal_rot),
            goal_approach,
        ), Setpoints.SCORE_CONE_HIGH
