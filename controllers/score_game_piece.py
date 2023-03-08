from controllers.arm import ArmController, get_setpoint_from_node
from components.intake import Intake
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController

from magicbot import state, StateMachine, feedback
from enum import Enum, auto
from utilities.game import (
    Node,
    get_closest_node,
    get_score_location,
    Rows,
    is_red,
    GamePiece,
    get_closest_node_in_allowed,
)
from components.score_tracker import GridNode, ScoreTracker


class NodePickStratergy(Enum):
    CLOSEST = auto()
    OVERRIDE = auto()
    BEST = auto()


def piece_to_node(piece: GamePiece) -> GridNode:
    if piece == GamePiece.BOTH:
        return GridNode.HYBRID
    if piece == GamePiece.CONE:
        return GridNode.CONE
    if piece == GamePiece.CUBE:
        return GridNode.CUBE


class ScoreGamePieceController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController

    movement: Movement
    recover: RecoverController

    score_tracker: ScoreTracker

    HARD_UP_SPEED = 0.3
    ARM_PRE_TIME = 1.5

    def __init__(self) -> None:
        self.node_stratergy = NodePickStratergy.BEST
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

    @timed_state(next_state="deploying_arm", duration=0.5, must_finish=True)
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
        if self.gripper.get_full_open():
            self.done()

    def done(self) -> None:
        super().done()
        self.recover.engage()

    def pick_node(self) -> Node:
        cur_pos = self.movement.chassis.get_pose().translation()
        if self.node_stratergy is NodePickStratergy.CLOSEST:
            return get_closest_node(
                cur_pos, self.gripper.get_current_piece(), self.prefered_row, []
            )
        elif self.node_stratergy is NodePickStratergy.OVERRIDE:
            return self.override_node
        elif self.node_stratergy is NodePickStratergy.BEST:
            state = (
                self.score_tracker.state_blue
                if is_red()
                else self.score_tracker.state_red
            )
            best = self.score_tracker.get_best_moves(
                state, piece_to_node(self.gripper.holding)
            )
            nodes: list[Node] = []
            for i in range(len(best)):
                as_tuple = tuple(best[i])
                node = Node(Rows(int(as_tuple[0])), as_tuple[1])
                nodes.append(node)

            return get_closest_node_in_allowed(
                cur_pos, self.gripper.get_current_piece(), nodes
            )

    @feedback
    def state_red(self) -> list[bool]:
        state: list[bool] = []
        for i in self.score_tracker.state_blue.tolist():
            for j in i:
                state.append(j)
        return state

    @feedback
    def state_blue(self) -> list[bool]:
        state: list[bool] = []
        for i in self.score_tracker.state_blue.tolist():
            for j in i:
                state.append(j)
        return state

    @feedback
    def pick_node_as_int(self) -> int:
        # node = self.pick_node()
        node = Node(Rows.HIGH, 0)
        return (node.row.value - 1) * 3 + node.col

    def prefer_high(self) -> None:
        self.prefered_row = Rows.HIGH

    def prefer_mid(self) -> None:
        self.prefered_row = Rows.MID
