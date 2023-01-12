from math import sqrt
from magicbot import StateMachine, state, timed_state, default_state
from components.chassis import Chassis
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryConfig
from wpimath.trajectory import TrajectoryGenerator
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.controller import HolonomicDriveController
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDControllerRadians
from math import pi
from math import atan2
from wpilib import Field2d

class Movement(StateMachine):
    chassis: Chassis
    field: Field2d

    def __init__(self):
        # create config
        self.goal = Pose2d(0, 0, pi)
        self.inputs = (0, 0, 0)
        self.drive_local = False
        self.config = TrajectoryConfig(1.5,1.5)

        self.x_controller = PIDController(3, 0, 0)
        self.y_controller = PIDController(3, 0, 0)
        self.heading_controller = ProfiledPIDControllerRadians(3, 0, 0,TrapezoidProfileRadians.Constraints(1,1))
        self.heading_controller.enableContinuousInput(0,2*pi)

        self.drive_controller = HolonomicDriveController(   
            self.x_controller,
            self.y_controller,
            self.heading_controller
        )
    
    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")

    # will execute if no other states are executing
    @default_state
    def manualdrive(self):
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self, state_tm, initial_call):
        
        #generate trajectory
        if initial_call:
            self.chassis_velocity = self.chassis.get_velocity()
            self.trajectory_preprocess_pose = Pose2d(
                self.chassis.get_pose().X(),
                self.chassis.get_pose().Y(),
                atan2(self.chassis_velocity.vy, self.chassis_velocity.vx)
            )
            self.config.setStartVelocity(sqrt((self.chassis_velocity.vy**2)+(self.chassis_velocity.vx**2)))
            self.auto_trajectory = TrajectoryGenerator.generateTrajectory(self.trajectory_preprocess_pose,[],self.goal,self.config)
            self.has_trajectory = True
            self.robot_object.setTrajectory(self.auto_trajectory)

        target_state = self.auto_trajectory.sample(state_tm)

        self.chassis_speed = self.drive_controller.calculate(self.chassis.get_pose(),target_state,self.goal.rotation())
        self.chassis.drive_local(self.chassis_speed.vx, self.chassis_speed.vy, self.chassis_speed.omega)
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
