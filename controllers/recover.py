from components.arm import Arm, Setpoints, MIN_EXTENSION
from components.intake import Intake
from components.gripper import Gripper

from magicbot import StateMachine, state


class RecoverController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm

    ARM_FOULING_ANGLE = 0.5

    def __init__(self) -> None:
        pass

    @state(first=True, must_finish=True)
    def retracting_arm(self) -> None:
        """
        Retract the arm to the minimum extension
        """
        self.arm.set_length(MIN_EXTENSION)
        if self.arm.at_goal_extension():
            self.next_state("clearing_intake")

    @state(must_finish=True)
    def clearing_intake(self) -> None:
        """
        Should check if the arm will foul on the intake
        mechanism. If there is an interception, the intake will be deployed
        and the arm will rotate above the intake
        """

        if self.arm.get_angle() > self.ARM_FOULING_ANGLE:
            self.intake.deploy_without_running()
            self.arm.go_to_setpoint(Setpoints.STOW)
        else:
            self.next_state("retracting_intake")

    @state(must_finish=True)
    def retracting_intake(self) -> None:
        """
        This stat will run for a single iteration of the robots control loop and
        is explicitly to clean up the intake mechanism if it was deployed while
        in the clearing state
        """
        self.intake.retract()
        self.next_state("stowing")

    @state(must_finish=True)
    def stowing(self) -> None:
        """
        This will bring the arm to the stow position
        """
        self.arm.go_to_setpoint(Setpoints.STOW)
        if self.arm.at_goal():
            self.done()
