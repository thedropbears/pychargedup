from components.arm import Arm
from components.intake import Intake
from components.gripper import Gripper

from magicbot import StateMachine, state


class RecoverController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm

    def __init__(self) -> None:
        pass

    @state(first=True, must_finish=True)
    def clearing_intake(self) -> None:
        """
        Initial state should check if the arm will foul on the intake
        mechanism. If there is an interception, the intake will be deployed
        and the arm will rotate above the intake
        """
        self.next_state("retracting_intake")
        
    @state(must_finish=True)
    def retracting_intake(self) -> None:
        """
        This stat will run for a single iteration of the robots control loop and
        is explicitly to clean up the intake mechanism if it was deployed while
        in the clearing state
        """
        self.next_state("stowing")

    @state(must_finish=True)
    def stowing(self) -> None:
        """ 
        This will bring the arm to the stow position
        """
        self.done()




