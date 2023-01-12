from magicbot import StateMachine, state, timed_state, default_state
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrajectoryConfig
from wpimath.trajectory import TrajectoryGenerator
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.controller import HolonomicDriveController
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.spline import Spline3
import math
from wpilib import Field2d

class Movement(StateMachine):
    chassis: Chassis
    field: Field2d

    def __init__(self):
        # create config
        self.goal = Pose2d(5,5,math.pi)
        print(self.goal.X(), self.goal.Y())
        self.goal_spline = Spline3.ControlVector((self.goal.X(), 0.2), (self.goal.Y(), 0.2))
        self.inputs = (0, 0, 0)
        self.drive_local = False
        self.config = TrajectoryConfig(1.5,1.5)

        self.x_controller = PIDController(3, 0, 0)
        self.y_controller = PIDController(3, 0, 0)
        self.heading_controller = ProfiledPIDControllerRadians(3, 0, 0,TrapezoidProfileRadians.Constraints(1,1))
        self.heading_controller.enableContinuousInput(0, math.tau)

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
                math.atan2(self.chassis_velocity.vy, self.chassis_velocity.vx)
            )
            self.trajectory_preprocess_vector = Spline3.ControlVector((self.chassis.get_pose().X(), self.chassis_velocity.vx*0.1), (self.chassis.get_pose().Y(), self.chassis_velocity.vy*0.1))

            self.config.setStartVelocity(math.sqrt((self.chassis_velocity.vy**2)+(self.chassis_velocity.vx**2)))

            self.auto_trajectory = TrajectoryGenerator.generateTrajectory(self.trajectory_preprocess_vector,[],self.goal_spline,self.config)
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
