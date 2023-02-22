from components.arm import Arm
from components.intake import Intake
from components.gripper import Gripper

from controllers.movement import Movement
from controllers.recover import RecoverController

from magicbot import state, StateMachine


class ScoreGamePieceController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm

    movement: Movement
    recover: RecoverController

    @state(first=True, must_finish=True)
    def driving_to_position(self):
        self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self):
        self.next_state("dropping")

    @state(must_finish=True)
    def dropping(self):
        self.done()

    def done(self) -> None:
        super().done()
        self.recover.engage()
