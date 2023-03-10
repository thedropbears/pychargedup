from controllers.arm import ArmController, get_setpoint_from_node
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController

from magicbot import state, timed_state, StateMachine
from utilities.game import Node, get_closest_node, get_score_location, Rows

from wpimath.geometry import Translation2d


class ScoreGamePieceController(StateMachine):
    gripper: Gripper
    arm: ArmController

    movement: Movement
    recover: RecoverController
    HARD_UP_SPEED = 0.3
    ARM_PRE_TIME = 1.5

    def __init__(self) -> None:
        self.override_node = Node(Rows.HIGH, 0)
        self.prefered_row = Rows.HIGH
        self.target_node = Node(Rows.HIGH, 0)

    @state(first=True, must_finish=True)
    def driving_to_position(self, initial_call) -> None:
        self.movement.set_goal(
            *get_score_location(self.target_node), max_accel=1.0, max_vel=2.0
        )
        self.movement.do_autodrive()
        if self.movement.is_at_goal():
            self.next_state("hard_up")

        if not initial_call and self.movement.time_to_goal < self.ARM_PRE_TIME:
            self.arm.go_to_setpoint(get_setpoint_from_node(self.target_node))

    @timed_state(next_state="deploying_arm", duration=0.3, must_finish=True)
    def hard_up(self) -> None:
        self.movement.inputs_lock = True
        self.movement.set_input(-self.HARD_UP_SPEED, 0, 0, False, override=True)

    @timed_state(next_state="deploying_arm", duration=0.5, must_finish=True)
    def back_off(self):
        if self.target_node.row is not Rows.MID:
            self.next_state("deploying_arm")
        self.movement.inputs_lock = True
        self.movement.set_input(self.HARD_UP_SPEED, 0, 0, False, override=True)

    @timed_state(next_state="open_flapper", duration=5.0, must_finish=True)
    def deploying_arm(self) -> None:
        self.arm.go_to_setpoint(get_setpoint_from_node(self.target_node))
        if self.arm.at_goal():
            self.next_state("open_flapper")

    @timed_state(next_state="dropping", duration=0.1, must_finish=True)
    def open_flapper(self) -> None:
        self.gripper.open_flapper()

    @timed_state(duration=0.5, must_finish=True)
    def dropping(self) -> None:
        self.gripper.open()

    def done(self) -> None:
        super().done()
        self.movement.inputs_lock = False
        self.recover.engage()

    def set_score_high(self) -> None:
        self.prefered_row = Rows.HIGH

    def set_score_mid(self) -> None:
        self.prefered_row = Rows.MID

    def _get_closest(self, row: Rows) -> Node:
        cur_pos = self.movement.chassis.get_pose().translation()
        cur_vel = self.movement.chassis.get_velocity()
        lookahead_time = 0.25
        effective_pos = cur_pos + Translation2d(
            cur_vel.vx * lookahead_time, cur_vel.vy * lookahead_time
        )
        return get_closest_node(effective_pos, self.gripper.get_current_piece(), row)

    def score(self) -> None:
        self.target_node = self._get_closest(self.prefered_row)
        self.engage()

    def score_without_moving(self, node: Node) -> None:
        self.target_node = node
        self.engage("deploying_arm", force=True)
