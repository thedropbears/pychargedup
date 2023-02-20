from magicbot import state, StateMachine, tunable, feedback
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import wpilib
import random

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm, Setpoints, Setpoint
from controllers.movement import Movement
from utilities.game import (
    GamePiece,
    field_flip_pose2d,
    get_double_substation,
    get_single_substation,
    field_flip_rotation2d,
    BLUE_NODES,
    Rows,
    Node,
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
    PICKUP_CUBE_TIMEOUT = tunable(0.2)

    desired_row = tunable(0)
    desired_column = tunable(0)

    # swap the the side of the substation to test on a half field
    swap_substation = tunable(False)

    snap_to_closest_node = tunable(True)

    def __init__(self) -> None:
        self.is_holding = GamePiece.NONE
        self.autodrive = False
        self.wants_piece = GamePiece.CONE
        self.cube_stack: list[tuple[Pose2d, Rotation2d]] = []
        self.score_stack: list[Node] = []
        self.wants_to_intake = False
        # use double substation shelf on right side from drivers pov
        self.driver_requests_right = False

    def get_correct_autodrive_state(self) -> str:
        cur_piece = self.get_current_piece()
        if cur_piece is GamePiece.CONE or cur_piece is GamePiece.CUBE:
            return "score"
        elif self.wants_piece is GamePiece.CONE:
            return "auto_pickup_cone"
        elif self.wants_piece is GamePiece.CUBE:
            return "auto_pickup_cube"
        return "idle"

    def go_to_autodrive_state(self):
        self.next_state(self.get_correct_autodrive_state())

    @state(first=True)
    def initialize(self, initial_call: bool, state_tm: float) -> None:
        if initial_call:
            self.arm.homing = True
        self.gripper.set_solenoid = False
        if self.arm.get_angle() > 0.5:
            self.intake.deploy_without_running()
        if self.arm.is_retracted():
            self.arm.set_at_min_extension()
            self.arm.homing = False
            self.arm.reset_controllers()
            self.arm.go_to_setpoint(Setpoints.STOW)

        if self.arm.get_angle() < 0.5 and not self.arm.homing:
            self.next_state("idle")
            self.gripper.set_solenoid = True

    @state
    def idle(self, initial_call: bool):
        if self.autodrive:
            self.go_to_autodrive_state()
        if self.wants_to_intake:
            self.next_state("intaking")
        if initial_call:
            self.arm.go_to_setpoint(Setpoints.STOW)
        self.intake.retract()

    @state
    def intaking(self):
        if self.autodrive:
            self.go_to_autodrive_state()
        if not self.wants_to_intake:
            self.next_state("idle")
            self.intake.retract()
            return
        self.arm.go_to_setpoint(Setpoints.HANDOFF)
        self.gripper.open()
        self.intake.deploy()
        if self.intake.is_game_piece_present():
            self.next_state("grab_from_well")

    @state
    def grab_from_well(self):
        """Grabs a cube from the well, assumes there is a cube in the well"""
        self.gripper.close()
        if self.gripper.get_full_closed():
            self.is_holding = GamePiece.CUBE
            self.wants_to_intake = False
            if self.cube_stack:
                self.cube_stack.pop()
            self.next_state("idle")

    @state
    def auto_pickup_cube(self):
        """Drives to intake a cube from the ground"""
        if not self.autodrive:
            self.next_state("idle")

        self.movement.set_goal(*self.get_cube_pickup(), slow_dist=0)
        if self.movement.time_to_goal < self.PICKUP_CUBE_PREPARE_TIME:
            self.arm.go_to_setpoint(Setpoints.HANDOFF)
            self.gripper.open()
            self.intake.deploy()
            if (
                self.intake.is_game_piece_present()
                or self.movement.time_to_goal < -self.PICKUP_CUBE_TIMEOUT
            ):
                self.next_state("grab_from_well")
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.go_to_setpoint(Setpoints.STOW)
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
                self.arm.go_to_setpoint(Setpoints.STOW)
        elif self.movement.time_to_goal < self.PICKUP_CONE_PREPARE_TIME:
            self.arm.go_to_setpoint(Setpoints.PICKUP_CONE)
            self.gripper.open()
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.go_to_setpoint(Setpoints.STOW)

        self.movement.do_autodrive()

    @state
    def score(self, initial_call, tm):
        if not self.autodrive:
            self.next_state("idle")

        if initial_call:
            move_goal, self.arm_setpoint = self.get_score_location()
            self.movement.set_goal(*move_goal)

        self.intake.retract()
        if self.movement.time_to_goal < self.SCORE_PRE_TIME and self.arm.at_goal():
            self.gripper.open()
            self.is_holding = GamePiece.NONE
            if self.gripper.get_full_open():
                print("scored at", tm)
                self.next_state("idle")
                self.arm.go_to_setpoint(Setpoints.STOW)
                if len(self.score_stack):
                    self.score_stack.pop()
                self.desired_column = random.randint(0, 8)
        elif self.movement.time_to_goal < self.SCORE_PREPARE_TIME:
            self.arm.go_to_setpoint(self.arm_setpoint)
        elif self.movement.time_to_goal > self.AUTO_STOW_CUTOFF:
            self.arm.go_to_setpoint(Setpoints.STOW)
        self.movement.do_autodrive()

    @feedback
    def is_red(self) -> bool:
        return self.get_team() == wpilib.DriverStation.Alliance.kRed

    def get_team(self) -> wpilib.DriverStation.Alliance:
        return wpilib.DriverStation.getAlliance()

    def get_current_piece(self) -> GamePiece:
        """What piece the gripper is currently holding in the gripper"""
        if self.gripper.get_full_closed():
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
        if not self.cube_stack:
            return self.get_single_substation_cube_pickup()
        else:
            return self.get_auto_cube_pickup()

    def get_single_substation_cube_pickup(self) -> tuple[Pose2d, Rotation2d]:
        goal_trans = get_single_substation(wpilib.DriverStation.Alliance.kBlue)
        goal_rotation = (
            field_flip_rotation2d(Rotation2d(0)) if self.is_red() else Rotation2d(0)
        )
        return Pose2d(goal_trans, goal_rotation), goal_rotation

    def get_auto_cube_pickup(self) -> tuple[Pose2d, Rotation2d]:
        return self.cube_stack[-1]

    def get_cone_pickup(self) -> tuple[Pose2d, Rotation2d]:
        # use the shelf closest to the wall, furthest from the nodes
        is_wall_side = self.driver_requests_right == self.is_red()
        # if we want the substation to be as if we are on the red alliance
        is_red = self.is_red() != self.swap_substation
        cone_trans = get_double_substation(is_red, is_wall_side).toTranslation2d()

        # as if we're blue
        goal_rotation = Rotation2d.fromDegrees(180)
        goal_approach = Rotation2d(0)
        offset_x, _ = Setpoints.PICKUP_CONE.toCartesian()
        if is_red:
            offset_x *= -1
            goal_rotation = field_flip_rotation2d(goal_rotation)
            goal_approach = field_flip_rotation2d(goal_approach)
        goal_trans = cone_trans + Translation2d(offset_x, 0)

        return (
            Pose2d(
                goal_trans,
                goal_rotation,
            ),
            goal_approach,
        )

    def set_direction_to_find_place(self, driver_requests_right: bool) -> None:
        """setting the side of the robot wants to get a cone or cube, also setting what side it wants to score"""
        self.driver_requests_right = driver_requests_right

    def get_score_location(self) -> tuple[tuple[Pose2d, Rotation2d], Setpoint]:
        if self.score_stack:  # is auto
            node = self.score_stack[-1]
        else:
            if self.snap_to_closest_node:
                desired_rows = Rows.HIGH if self.driver_requests_right else Rows.MID
                score_locations = []
                if self.get_current_piece() == GamePiece.CONE:
                    score_locations = [
                        self.score_location_from_node(
                            Node(desired_rows, i), self.is_red()
                        )
                        for i in range(9)
                        if i % 3 != 1
                    ]
                elif self.get_current_piece == GamePiece.CUBE:
                    score_locations = [
                        self.score_location_from_node(
                            Node(desired_rows, i), self.is_red()
                        )
                        for i in range(1, 9, 3)
                    ]
                score_poses = [t[0][0] for t in score_locations]
                if len(score_poses) == 0:
                    node = Node(Rows(self.desired_row), self.desired_column)
                nearest = self.movement.chassis.get_pose().nearest(score_poses)
                return next(loc for loc in score_locations if loc[0][0] == nearest)
            else:
                node = Node(Rows(self.desired_row), self.desired_column)
        return self.score_location_from_node(node, self.is_red())

    def score_location_from_node(
        self, node: Node, is_red: bool
    ) -> tuple[tuple[Pose2d, Rotation2d], Setpoint]:
        node_trans3d = BLUE_NODES[node.row.value][int(node.col)]
        node_trans = node_trans3d.toTranslation2d()

        setpoint = Setpoints.SCORE_CONE_HIGH
        if self.get_current_piece() == GamePiece.CONE:
            if node.row == Rows.HIGH:
                setpoint = Setpoints.SCORE_CONE_HIGH
            elif node.row == Rows.MID:
                setpoint = Setpoints.SCORE_CONE_MID
        elif self.get_current_piece() == GamePiece.CUBE:
            if node.row == Rows.HIGH:
                setpoint = Setpoints.SCORE_CUBE_HIGH
            elif node.row == Rows.MID:
                setpoint = Setpoints.SCORE_CUBE_MID

        offset_x, _ = setpoint.toCartesian()
        goal_trans = node_trans + Translation2d(-offset_x, 0)
        goal = Pose2d(goal_trans, Rotation2d(0))
        goal_approach = Rotation2d.fromDegrees(180)
        if is_red:
            goal = field_flip_pose2d(goal)
            goal_approach = field_flip_rotation2d(goal_approach)

        return (goal, goal_approach), setpoint
