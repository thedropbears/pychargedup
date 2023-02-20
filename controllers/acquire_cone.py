from magicbot import state, StateMachine, tunable
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from components.gripper import Gripper
from components.intake import Intake
from components.arm import Arm, Setpoints
from controllers.movement import Movement
from utilities.game import (
    get_double_substation,
    field_flip_rotation2d,
)


class AcquireConeController(StateMachine):
    gripper: Gripper
    intake: Intake
    arm: Arm
    movement: Movement

    APPROACH_SPEED: float = tunable(0.2)

    def __init__(self) -> None:
        self.targeting_left = False

    @state(start=True, must_finish=True)
    def driving_to_position(self) -> None:
        """
        Get the chassis to the correct position to start moving towards cone.
        Requires that the state has had the goal position injected into it.
        """

        self.movement.set_goal(*self.get_cone_pickup())
        if self.movement.is_at_goal():
            self.next_state("deploying_arm")

    @state(must_finish=True)
    def deploying_arm(self) -> None:
        """
        Move the arm into position to pick up cone.
        Open the gripper
        """

        # TODO Test if gripper opening interfere with the arm moving from HANDOFF
        self.arm.set_angle()
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

        if self.gripper.game_piece_in_reach():
            self.next_state("grabbing")

    @state(must_finish=True)
    def grabbing(self) -> None:
        """
        Close the gripper on the cone.
        """
        self.done()

    def target_left(self) -> None:
        self.targeting_left = True

    def target_right(self) -> None:
        self.targeting_left = False

    def get_cone_pickup(self) -> tuple[Pose2d, Rotation2d]:
        # if we want the substation to be as if we are on the red alliance
        is_red = self.is_red() != self.swap_substation
        cone_trans = get_double_substation(
            is_red, self.targeting_left
        ).toTranslation2d()

        # as if we're blue
        goal_rotation = Rotation2d.fromDegrees(180)
        goal_approach = Rotation2d(0)
        offset_x = Setpoints.PICKUP_CONE.toCartesian()
        if is_red:
            offset_x *= -1
            goal_rotation = field_flip_rotation2d(goal_rotation)
            goal_approach = field_flip_rotation2d(goal_approach)
        goal_trans = cone_trans + Translation2d(offset_x, 0)

        return (
            Pose2d(
                goal_trans,
                goal_rotation,
            ),
            goal_approach,
        )
