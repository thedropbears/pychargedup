from components.arm import Arm
from components.intake import Intake
from components.gripper import Gripper
from controllers.movement import Movement

from magicbot import state, StateMachine

class AcquireConeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    @state(start=True, must_finish=True)
    def driving_to_position(self) -> None:
        """
        Get the chassis to the correct position to start moving towards cone.
        Requires that the state has had the goal position injected into it.
        """
        self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self) -> None:
        """
        Move the arm into position to pick up cone.
        """
        self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        """
        Extend the arm towards the cone until the limit switch is triggered on the wall.
        """
        self.next_state("grabbing")

    @state(must_finish=True)
    def grabbing(self) -> None:
        """
        Close the gripper on the cone.
        """
        self.done()
