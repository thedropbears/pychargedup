from controllers.arm import ArmController, Setpoints
from components.intake import Intake
from components.gripper import Gripper

from magicbot import StateMachine, state, timed_state


class RecoverController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController

    ARM_FOULING_ANGLE = 0.55

    def __init__(self) -> None:
        self.has_initialized_arm = False

    @state(first=True, must_finish=True)
    def retracting_arm(self):
        """
        Retract the arm to the minimum extension
        """
        if self.has_initialized_arm:
            self.arm.arm_component.set_use_voltage(False)
            self.next_state("clearing_intake")
            return
        if self.arm.is_at_retraction_limit():
            self.has_initialized_arm = True
            self.arm.arm_component.set_at_min_extension()
        self.arm.arm_component.set_voltage(-2.0)
        self.arm.arm_component.set_use_voltage(True)

    @timed_state(duration=0.4, next_state="stowing_arm", must_finish=True)
    def clearing_intake(self) -> None:
        """
        Should check if the arm will foul on the intake
        mechanism. If there is an interception, the intake will be deployed
        and the arm will rotate above the intake
        """

        if self.arm.get_angle() > self.ARM_FOULING_ANGLE and not self.intake.deployed:
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
