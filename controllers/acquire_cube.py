from controllers.arm import ArmController, Setpoints
from components.intake import Intake
from components.gripper import Gripper
from components.leds import StatusLights

from controllers.recover import RecoverController

from magicbot import state, StateMachine

from utilities.game import GamePiece


class AcquireCubeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController
    status_lights: StatusLights

    recover: RecoverController

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        """
        Deploy the intake.
        Open the gripper.
        """
        self.intake.deploy()
        self.status_lights.want_cube()
        self.gripper.open()
        if self.gripper.get_full_open() and self.intake.is_fully_deployed():
            self.next_state("moving_arm")

    @state(must_finish=True)
    def moving_arm(self) -> None:
        """
        Put the arm in the right place to receive a cube.
        """
        self.arm.go_to_setpoint(Setpoints.HANDOFF)
        if self.arm.at_goal():
            self.next_state("waiting_for_cube")

    @state(must_finish=True)
    def waiting_for_cube(self) -> None:
        """
        Keep the intake spinning until the cube breaks the break-beam sensor.
        """
        if self.intake.is_game_piece_present():
            self.intake.deploy_without_running()
            self.next_state("grabbing")

    @state(must_finish=True)
    def clamping(self) -> None:
        """bring the intake in so the cube piece is in a known position"""
        self.intake.retract()
        if self.intake.is_fully_retracted():
            self.next_state("grabbing")

    @state(must_finish=True)
    def grabbing(self) -> None:
        """
        Close the gripper on the cube.
        Recovery controller will take care of moving the arm to stow, and retracting the intake.
        """
        self.gripper.close(GamePiece.CUBE)
        if self.gripper.get_full_closed():
            self.status_lights.cube_onboard()
            self.next_state("raising")

    @state(must_finish=True)
    def raising(self):
        self.arm.go_to_setpoint(Setpoints.STOW, do_retract=False)
        if self.arm.at_goal():
            self.done()

    def done(self) -> None:
        super().done()
        self.recover.engage()
