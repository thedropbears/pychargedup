from components.arm import Arm, Setpoint, Setpoints

from magicbot import StateMachine, state


class ArmController(StateMachine):
    arm: Arm

    def __init__(self) -> None:
        self._target_setpoint: Setpoint = Setpoints.STOW

    def go_to_setpoint(self, setpoint: Setpoint) -> None:
        # Only restart the state machine if the setpoint is different
        if setpoint != self._target_setpoint:
            self.done()
            self._target_setpoint = setpoint
            self.engage()

    def get_angle(self) -> float:
        return self.arm.get_angle()

    def at_goal(self) -> bool:
        return not self.is_executing

    @state(first=True, must_finish=True)
    def retracting_arm(self) -> None:
        self.next_state("rotating_arm")

    @state(must_finish=True)
    def rotating_arm(self) -> None:
        self.next_state("extending_arm")

    @state(must_finish=True)
    def extending_arm(self) -> None:
        self.done()
