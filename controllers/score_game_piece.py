from components.arm import Arm, get_setpoint_from_node
from components.intake import Intake
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController

from magicbot import state, StateMachine
from enum import Enum, auto
from utilities.game import Node, get_closest_node, get_score_location, Rows


class NodePickStratergy(Enum):
    CLOSEST = auto()
    OVERRIDE = auto()
    BEST = auto()


class ScoreGamePieceController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm

    movement: Movement
    recover: RecoverController

    def __init__(self) -> None:
        self.node_stratergy = NodePickStratergy.CLOSEST
        self.override_node = Node(Rows.HIGH, 0)
        self.target_node = Node(Rows.HIGH, 0)

    @state(first=True, must_finish=True)
    def driving_to_position(self, initial_call):
        if initial_call:
            self.target_node = self.pick_node()
        self.movement.set_goal(*get_score_location(self.target_node))
        if self.movement.is_at_goal():
            self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self):
        self.arm.go_to_setpoint(get_setpoint_from_node(self.target_node))
        if self.arm.at_goal():
            self.next_state("dropping")

    @state(must_finish=True)
    def dropping(self):
        self.gripper.open()
        if self.gripper.get_full_open():
            self.done()

    def done(self) -> None:
        super().done()
        self.recover.engage()

    def pick_node(self) -> Node:
        cur_pos = self.movement.chassis.get_pose().translation()
        if self.node_stratergy is NodePickStratergy.CLOSEST:
            return get_closest_node(cur_pos, self.gripper.get_current_piece())
        elif self.node_stratergy is NodePickStratergy.OVERRIDE:
            return self.override_node
        elif self.node_stratergy is NodePickStratergy.BEST:
            return get_closest_node(cur_pos, self.gripper.get_current_piece())
