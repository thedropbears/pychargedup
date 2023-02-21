from magicbot import state, StateMachine, tunable

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm, Setpoints
from controllers.movement import Movement
from utilities.game import (
    get_cone_pickup,
)


class AcquireConeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    APPROACH_SPEED = tunable(0.2)

    def __init__(self) -> None:
        self.targeting_left: bool = False

    @state(first=True, must_finish=True)
    def driving_to_position(self) -> None:
        """
        Get the chassis to the correct position to start moving towards cone.
        Requires that the state has had the goal position injected into it.
        """

        self.movement.set_goal(*get_cone_pickup(self.targeting_left))
        if self.movement.is_at_goal():
            self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self) -> None:
        """
        Move the arm into position to pick up cone.
        Open the gripper
        """

        # TODO Test if gripper opening interfere with the arm moving from HANDOFF
        self.arm.go_to_setpoint(Setpoints.PICKUP_CONE)
        if self.arm.at_goal():
            self.gripper.open()
        if self.gripper.get_full_open():
            self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        """
        Move forward until the limit switch is triggered on the wall.
        """

        self.arm.extend(0.2)
        if self.gripper.game_piece_in_reach():
            self.next_state("grabbing")

    @state(must_finish=True)
    def grabbing(self) -> None:
        """
        Close the gripper on the cone.
        """
        self.gripper.close()
        if self.gripper.get_full_closed():
            self.done()

    def target_left(self) -> None:
        self.targeting_left = True

    def target_right(self) -> None:
        self.targeting_left = False
