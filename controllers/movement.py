from magicbot import StateMachine, state, timed_state, default_state
from wpimath.geometry import Pose2d
from components.chassis import Chassis
from utilities.functions import clamp_2d


class Movement(StateMachine):
    chassis: Chassis

    def __init__(self):
        self.goal = Pose2d()
        self.inputs = (0, 0, 0)
        self.drive_local = False

    # will execute if no other states are executing
    @default_state
    def manualdrive(self):
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self):
        """Drive to a waypoint"""
        ...

    @state
    def score(self):
        ...

    @state
    def pickup(self):
        ...

    @state
    def comfirm_action(self):
        ...

    def set_input(self, vx: float, vy: float, vz: float, local: bool):
        # clamp to inputs to be achiveable
        self.inputs = (*clamp_2d((vx, vy), Chassis.max_wheel_speed), vz)
        self.drive_local = local

    def do_autodrive(self):
        self.engage()
