from magicbot import state, StateMachine, tunable

from components.gripper import Gripper
from components.intake import Intake
from controllers.arm import ArmController, Setpoints
from components.leds import StatusLights

from controllers.movement import Movement
from controllers.recover import RecoverController

from utilities.game import (
    GamePiece,
    get_cone_pickup,
    is_red,
)


class AcquireConeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: ArmController
    status_lights: StatusLights

    movement: Movement
    recover: RecoverController

    EXTEND_VOLTAGE = tunable(3)

    # swap the the side of the substation to test on a half field
    swap_substation = tunable(False)

    def __init__(self) -> None:
        self.targeting_left: bool = False

    @state(first=True, must_finish=True)
    def driving_to_position(self) -> None:
        """
        Get the chassis to the correct position to start moving towards cone.
        Requires that the state has had the goal position injected into it.
        """
        # if we want the substation to be as if we are on the red alliance
        red_side = is_red() != self.swap_substation
        stop_distance = 0.1
        x_offset = (
            Setpoints.PREPARE_PICKUP_CONE.toCartesian()[0]
            + self.arm.arm_component.PIVOT_X
            + stop_distance
        )
        self.movement.set_goal(
            *get_cone_pickup(self.targeting_left, red_side, x_offset)
        )
        self.movement.do_autodrive()
        if self.movement.is_at_goal():
            self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self) -> None:
        """
        Move the arm into position to pick up cone.
        Open the gripper
        """

        # TODO Test if gripper opening interfere with the arm moving from HANDOFF
        self.arm.go_to_setpoint(Setpoints.PREPARE_PICKUP_CONE)
        if self.arm.at_goal():
            self.gripper.open()
        if self.gripper.get_full_open():
            self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        """
        Move forward until the limit switch is triggered on the wall.
        """

        self.arm.go_to_setpoint(Setpoints.PICKUP_CONE)
        if self.arm.is_at_forward_limit():
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
