from magicbot import StateMachine, state, timed_state, default_state
from wpimath.geometry import Pose2d
from components.chassis import Chassis
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class Movement(StateMachine):
    chassis: Chassis

    def __init__(self):
        self.goal = Pose2d()
        self.inputs = (0, 0, 0)
        self.drive_local = False
        self.x_controller = ProfiledPIDController(
            3, 0, 0, TrapezoidProfile.Constraints(1, 1)
        )
        self.y_controller = ProfiledPIDController(
            3, 0, 0, TrapezoidProfile.Constraints(1, 1)
        )
        self.heading_controller = ProfiledPIDController(
            3, 0, 0, TrapezoidProfile.Constraints(1, 1)
        )

    # will execute if no other states are executing
    @default_state
    def manualdrive(self):
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self):
        self.x_controller.setGoal(self.goal.X())
        self.y_controller.setGoal(self.goal.Y())
        self.heading_controller.setGoal(self.goal.rotation().radians())

        x_velocity = self.x_controller.calculate(self.chassis.get_pose().X())
        y_velocity = self.y_controller.calculate(self.chassis.get_pose().Y())
        omega = self.heading_controller.calculate(
            self.chassis.get_pose().rotation().radians()
        )

        self.chassis.drive_field(x_velocity, y_velocity, omega)

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
        """Sets teleoperated drive inputs"""
        self.inputs = (vx, vy, vz)
        self.drive_local = local

    def do_autodrive(self):
        self.engage()
