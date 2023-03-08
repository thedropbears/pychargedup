from magicbot import state, StateMachine

from components.gripper import Gripper
from components.intake import Intake
from controllers.arm import ArmController, Setpoints
from components.leds import StatusLights

from controllers.movement import Movement
from controllers.recover import RecoverController

from utilities.game import (
    GamePiece,
    get_cone_pickup,
)


class AcquireConeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController
    status_lights: StatusLights

    movement: Movement
    recover: RecoverController

    ARM_PRE_TIME = 1

    def __init__(self) -> None:
        self.targeting_left: bool = False

    @state(first=True, must_finish=True)
    def driving_to_position(self) -> None:
        """
        Get the chassis to the correct position to start moving towards cone.
        Requires that the state has had the goal position injected into it.
        """
        self.movement.set_goal(
            *get_cone_pickup(self.movement.chassis.get_pose(), self.targeting_left)
        )
        self.movement.do_autodrive()
        if self.movement.is_at_goal():
            self.next_state("deploying_arm")

        if self.movement.time_to_goal < self.ARM_PRE_TIME:
            self.arm.go_to_setpoint(Setpoints.PREPARE_PICKUP_CONE)

    @state(must_finish=True)
    def deploying_arm(self) -> None:
        """
        Move the arm into position to pick up cone.
        Open the gripper
        """

        self.arm.go_to_setpoint(Setpoints.PREPARE_PICKUP_CONE)
        self.gripper.open()
        if self.gripper.get_full_open() and self.arm.at_goal():
            self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        """
        Move forward until the limit switch is triggered on the wall.
        """

        self.arm.go_to_setpoint(Setpoints.PICKUP_CONE)
        if self.arm.is_at_forward_limit() or self.arm.at_goal():
            self.next_state("grabbing")

    @state(must_finish=True)
    def grabbing(self) -> None:
        """
        Close the gripper on the cone.
        """
        self.gripper.close(GamePiece.CONE)
        if self.gripper.get_full_closed():
            self.status_lights.cone_onboard()
            self.done()

    def done(self) -> None:
        super().done()
        self.recover.engage()

    def target_left(self) -> None:
        self.targeting_left = True
        self.status_lights.want_cone_left()

    def target_right(self) -> None:
        self.targeting_left = False
        self.status_lights.want_cone_right()
