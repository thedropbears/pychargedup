from magicbot import StateMachine, state, default_state
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    Trajectory,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.trajectory.constraint import (
    CentripetalAccelerationConstraint,
    RectangularRegionConstraint,
    MaxVelocityConstraint,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.spline import Spline3
import math
from wpilib import Field2d


class Movement(StateMachine):
    chassis: Chassis
    field: Field2d

    def __init__(self) -> None:
        # create config
        self.inputs = (0, 0, 0)
        self.drive_local = False

        self.goal = Pose2d(3, 0, 0)
        self.goal_spline = Spline3.ControlVector(
            (self.goal.X(), -6), (self.goal.Y(), 0)
        )
        self.goal_rotation = Rotation2d()
        

        self.debug_trajectory = True
        self.config = TrajectoryConfig(1, 1.5)
        self.config.addConstraint(CentripetalAccelerationConstraint(1.5))
        topRight = Translation2d(self.goal.X() + 2, self.goal.Y() - 2)
        bottomLeft = Translation2d(self.goal.X() - 2, self.goal.Y() + 2)
        self.config.addConstraint(
            RectangularRegionConstraint(bottomLeft, topRight, MaxVelocityConstraint(1))
        )

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")

    def generate_trajectory(self) -> Trajectory:
        self.chassis_velocity = self.chassis.get_velocity()
        self.x_pos = self.chassis.get_pose().X()
        self.y_pos = self.chassis.get_pose().Y()

        self.x_velocity = self.chassis_velocity.vx
        self.y_velocity = self.chassis_velocity.vy

        self.trajectory_preprocess_pose = Pose2d(
            self.x_pos, self.y_pos, math.atan2(self.y_velocity, self.x_velocity)
        )

        x_translation = self.goal.X() - self.x_pos
        y_translation = self.goal.Y() - self.y_pos

        translation_distance = 0
        translation_distance = math.sqrt(x_translation**2 + y_translation**2)
       
        if translation_distance == 0:
            return Trajectory([Trajectory.State(0, 0, 0, self.chassis.get_pose())])

        if math.sqrt(self.x_velocity**2 + self.y_velocity**2) < 0.2:

            kD = 0.3

            self.spline_start_momentum_x = x_translation * kD
            self.spline_start_momentum_y = y_translation * kD

        else:
            kvx = 3
            kvy = 3

            self.spline_start_momentum_x = self.chassis_velocity.vx * kvx
            self.spline_start_momentum_y = self.chassis_velocity.vy * kvy

        k_spline = min(8, translation_distance*2)

        self.goal_spline = Spline3.ControlVector(
            (self.goal.X(), self.goal_rotation.cos() * k_spline),
            (self.goal.Y(), self.goal_rotation.sin() * k_spline),
        )

        self.trajectory_preprocess_vector = Spline3.ControlVector(
            (self.x_pos, self.spline_start_momentum_x),
            (self.y_pos, self.spline_start_momentum_y),
        )

        self.config.setStartVelocity(
            math.sqrt(self.y_velocity**2 + self.x_velocity**2)
        )
    
        self.auto_trajectory = TrajectoryGenerator.generateTrajectory(
            self.trajectory_preprocess_vector, [], self.goal_spline, self.config
        )

        self.robot_object.setTrajectory(self.auto_trajectory)
        return self.auto_trajectory

    def set_goal(self, goal: Pose2d, approach_direction: Rotation2d) -> None:
        self.goal = goal
        self.goal_rotation = approach_direction

        self.config = TrajectoryConfig(1, 1.5)
        self.config.addConstraint(CentripetalAccelerationConstraint(1.5))
        topRight = Translation2d(self.goal.X() + 2, self.goal.Y() - 2)
        bottomLeft = Translation2d(self.goal.X() - 2, self.goal.Y() + 2)
        self.config.addConstraint(
            RectangularRegionConstraint(bottomLeft, topRight, MaxVelocityConstraint(1))
        )

    # will execute if no other states are executing
    @default_state
    def manualdrive(self) -> None:
        if self.debug_trajectory == True:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self, state_tm: float, initial_call: bool) -> None:
        # generate trajectory
        if initial_call:
            self.x_controller = PIDController(2.5, 0, 0)
            self.y_controller = PIDController(2.5, 0, 0)
            self.heading_controller = ProfiledPIDControllerRadians(
                5, 0, 0, TrapezoidProfileRadians.Constraints(1, 1)
            )
            self.heading_controller.enableContinuousInput(0, math.tau)

            self.drive_controller = HolonomicDriveController(
                self.x_controller, self.y_controller, self.heading_controller
            )
            self.trajectory = self.generate_trajectory()

        target_state = self.trajectory.sample(state_tm)

        self.chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(), target_state, self.goal.rotation()
        )
        self.chassis.drive_local(
            self.chassis_speed.vx, self.chassis_speed.vy, self.chassis_speed.omega
        )

    @state
    def score(self) -> None:
        ...

    @state
    def pickup(self) -> None:
        ...

    @state
    def comfirm_action(self) -> None:
        ...

    def set_input(self, vx: float, vy: float, vz: float, local: bool):
        """Sets teleoperated drive inputs"""
        self.inputs = (vx, vy, vz)
        self.drive_local = local

    def do_autodrive(self, goal: Pose2d, approach_direction: Rotation2d) -> None:
        self.set_goal(goal,approach_direction)
        self.engage()
