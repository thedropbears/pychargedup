from controllers.arm import ArmController, get_setpoint_from_node
from components.intake import Intake
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController

from magicbot import state, timed_state, StateMachine
from enum import Enum, auto
from utilities.game import Node, get_closest_node, get_score_location, Rows

from wpimath.geometry import Translation2d


class NodePickStratergy(Enum):
    CLOSEST = auto()
    OVERRIDE = auto()
    BEST = auto()


class ScoreGamePieceController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController

    movement: Movement
    recover: RecoverController
    HARD_UP_SPEED = 0.3
    ARM_PRE_TIME = 2.0

    def __init__(self) -> None:
        self.node_stratergy = NodePickStratergy.CLOSEST
        self.override_node = Node(Rows.HIGH, 0)
        self.prefered_row = Rows.HIGH
        self.target_node = Node(Rows.HIGH, 0)

    @state(first=True, must_finish=True)
    def driving_to_position(self) -> None:
        self.movement.set_goal(*get_score_location(self.target_node))
        self.movement.do_autodrive()
        if self.movement.is_at_goal():
            self.next_state("hard_up")

        if self.movement.time_to_goal < self.ARM_PRE_TIME:
            self.arm.go_to_setpoint(get_setpoint_from_node(self.target_node))

    @timed_state(next_state="back_off", duration=0.3, must_finish=True)
    def hard_up(self) -> None:
        self.movement.inputs_lock = True
        self.movement.set_input(-self.HARD_UP_SPEED, 0, 0, False, override=True)

    @timed_state(next_state="deploying_arm", duration=0.4, must_finish=True)
    def back_off(self):
        if self.target_node.row is not Rows.MID:
            self.next_state("deploying_arm")
        self.movement.inputs_lock = True
        self.movement.set_input(self.HARD_UP_SPEED, 0, 0, False, override=True)

    @state(must_finish=True)
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

    def score_closest_high(self) -> None:
        self.target_node = self._get_closest(Rows.HIGH)
        self.engage()

    def score_closest_mid(self) -> None:
        self.target_node = self._get_closest(Rows.MID)
        self.engage()

    def _get_closest(self, row: Rows) -> Node:
        cur_pos = self.movement.chassis.get_pose().translation()
        cur_vel = self.movement.chassis.get_velocity()
        lookahead_time = 1.0
        effective_pos = cur_pos + Translation2d(
            cur_vel.vx * lookahead_time, cur_vel.vy * lookahead_time
        )
        return get_closest_node(effective_pos, self.gripper.get_current_piece(), row)

    def score_best(self) -> None:
        # placeholder
        self.score_closest_high()

    def score_without_moving(self, node: Node) -> None:
        self.target_node = node
        self.engage("deploying_arm", force=True)
