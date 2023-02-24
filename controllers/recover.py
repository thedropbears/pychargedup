from controllers.arm import ArmController, Setpoints
from components.intake import Intake
from components.gripper import Gripper

from magicbot import StateMachine, state, timed_state


class RecoverController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController

    ARM_FOULING_ANGLE = 0.52

    def __init__(self) -> None:
        pass

    @timed_state(first=True, duration=3.0, next_state="stowing_arm", must_finish=True)
    def clearing_intake(self) -> None:
        """
        Should check if the arm will foul on the intake
        mechanism. If there is an interception, the intake will be deployed
        and the arm will rotate above the intake
        """

        if self.arm.get_angle() > self.ARM_FOULING_ANGLE:
            self.intake.deploy_without_running()
        else:
            self.next_state("stowing_arm")

    @state(must_finish=True)
    def stowing_arm(self) -> None:
        self.arm.go_to_setpoint(Setpoints.STOW)
        if self.arm.at_goal():
            self.next_state("retracting_intake")

    @state(must_finish=True)
    def retracting_intake(self) -> None:
        """
        This stat will run for a single iteration of the robots control loop and
        is explicitly to clean up the intake mechanism if it was deployed while
        in the clearing state
        """
        self.intake.retract()
        self.done()
