from magicbot import state, StateMachine, tunable, feedback
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import wpilib

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm, Setpoints, Setpoint
from controllers.movement import Movement
from utilities import field


class ScoringController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    grab_pre_time = tunable(0.5)

    def __init__(self) -> None:
        self.holding_piece = field.GamePiece.CONE
        self.autodrive = False
        self.wants_piece = field.GamePiece.CUBE
        self.wants_to_intake = False
        # use double substation shelf on right side from drivers pov
        self.cone_pickup_side_right = False

    def get_correct_state(self) -> str:
        if self.autodrive:
            cur_piece = self.get_current_piece()
            if cur_piece is field.GamePiece.CONE or cur_piece is field.GamePiece.CUBE:
                return "score"
            elif self.wants_piece is field.GamePiece.CONE:
                return "auto_pickup_cone"
            elif self.wants_piece is field.GamePiece.CUBE:
                return "auto_pickup_cube"
        else:
            if self.wants_to_intake:
                return "intaking"
        return "idle"

    def goto_correct_state(self):
        self.next_state_now(self.get_correct_state())

    @state(first=True)
    def idle(self, state_tm: float):
        self.goto_correct_state()
        # self.arm.stop()
        if state_tm > 0.5:
            self.arm.goto_setpoint(Setpoints.STOW)

    @state
    def intaking(self):
        self.goto_correct_state()

        self.arm.goto_setpoint(Setpoints.HANDOFF)
        self.intake.deploy()
        if self.gripper.game_piece_in_reach():
            self.gripper.close()
            self.holding_piece = field.GamePiece.CUBE

    @state
    def auto_pickup_cube(self, initial_call):
        """Drives to the single substation to intake a cube"""
        self.goto_correct_state()

        self.movement.set_goal(*self.get_cube_pickup())
        if self.movement.time_to_goal < 2:
            self.arm.goto_setpoint(Setpoints.HANDOFF)
            self.gripper.open()
            self.intake.deploy()
            if self.gripper.game_piece_in_reach():
                self.gripper.close()
                self.holding_piece = field.GamePiece.CUBE
        self.movement.do_autodrive()

    @state
    def auto_pickup_cone(self):
        self.goto_correct_state()

        self.movement.set_goal(*self.get_cone_pickup())
        self.intake.retract()
        if self.movement.time_to_goal < 2:
            self.arm.goto_setpoint(Setpoints.PICKUP_CONE)
            self.gripper.open()
        if self.movement.time_to_goal < self.grab_pre_time:
            self.gripper.close()
            self.holding_piece = field.GamePiece.CONE
        self.movement.do_autodrive()

    @state
    def score(self, initial_call):
        self.goto_correct_state()

        if initial_call:
            move_goal, self.arm_setpoint = self.get_score_location()
            self.movement.set_goal(*move_goal)

        self.intake.retract()
        if self.movement.time_to_goal < 4:
            self.arm.goto_setpoint(self.arm_setpoint)
        if self.movement.time_to_goal < 0.1:
            self.gripper.open()
        self.movement.do_autodrive()

    def is_red(self) -> bool:
        return wpilib.DriverStation.getAlliance() is wpilib.DriverStation.Alliance.kRed

    def get_team(self) -> wpilib.DriverStation.Alliance:
        return wpilib.DriverStation.getAlliance()

    def get_current_piece(self) -> field.GamePiece:
        """What piece the gripper is currently holding"""
        if self.gripper.get_full_closed():
            return self.holding_piece
        return field.GamePiece.NONE

    @feedback
    def get_current_piece_str(self) -> str:
        return self.get_current_piece().name

    @feedback
    def get_wants_piece_str(self) -> str:
        return self.wants_piece.name

    def get_cube_pickup(self) -> tuple[Pose2d, Rotation2d]:
        """Gets where to auto pickup cubes from"""
        # can be changed for autonomous period?
        goal_trans = field.get_single_substation(wpilib.DriverStation.Alliance.kBlue)
        goal_rotation = (
            field.field_flip_rotation2d(Rotation2d(0))
            if self.is_red()
            else Rotation2d(0)
        )
        return Pose2d(goal_trans, goal_rotation), goal_rotation

    def get_cone_pickup(self) -> tuple[Pose2d, Rotation2d]:
        is_wall_side = self.cone_pickup_side_right == self.is_red()
        goal_trans = field.get_double_substation(self.get_team(), is_wall_side)
        goal_rotation = (
            field.field_flip_rotation2d(Rotation2d(0))
            if self.is_red()
            else Rotation2d(0)
        )
        return (
            Pose2d(
                goal_trans.toTranslation2d(),
                goal_rotation + Rotation2d.fromDegrees(180),
            ),
            goal_rotation,
        )

    def get_score_location(self) -> tuple[tuple[Pose2d, Rotation2d], Setpoint]:
        # TODO: pick actual scoring node
        goal_trans = field.field_flip_translation2d(
            Translation2d(field.GRIDS_EDGE_X + 0.5, 2)
        )
        goal_rot = field.field_flip_rotation2d(Rotation2d.fromDegrees(180))
        return (
            Pose2d(goal_trans, goal_rot + Rotation2d.fromDegrees(180)),
            goal_rot,
        ), Setpoints.SCORE_CONE_HIGH
