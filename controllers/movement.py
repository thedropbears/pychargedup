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

        self.goal = Pose2d(3, 0, math.pi)
        self.goal_spline = Spline3.ControlVector(
            (self.goal.X(), -6), (self.goal.Y(), 0)
        )

        self.debug_trajectory = False
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

        if math.hypot(self.x_velocity, self.y_velocity) < 0.2:
            # Normalisation
            x_translation = self.goal.X() - self.x_pos
            y_translation = self.goal.Y() - self.y_pos

            translation_distance = math.sqrt(x_translation**2 + y_translation**2)

            if translation_distance == 0:
                return Trajectory([Trajectory.State(0, 0, 0, self.chassis.get_pose())])

            normalised_x = x_translation / translation_distance
            normalised_y = y_translation / translation_distance

            kD = 5

            self.spline_start_momentum_x = normalised_x * kD
            self.spline_start_momentum_y = normalised_y * kD

        else:
            kvx = 3
            kvy = 3

            self.spline_start_momentum_x = self.chassis_velocity.vx * kvx
            self.spline_start_momentum_y = self.chassis_velocity.vy * kvy

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

    def set_goal(self, goal: Pose2d, approach_direciton: Rotation2d) -> None:
        self.goal = goal
        self.goal_spline = Spline3.ControlVector(
            (self.goal.X(), approach_direciton.cos() * 1),
            (self.goal.Y(), approach_direciton.sin() * 1),
        )

        self.config = TrajectoryConfig(1, 1.5)
        self.config.addConstraint(CentripetalAccelerationConstraint(1.5))
        topRight = Translation2d(self.goal.X() + 3, self.goal.Y() - 3)
        bottomLeft = Translation2d(self.goal.X() - 3, self.goal.Y() + 3)
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
            self.x_controller = PIDController(1.5, 0, 0)
            self.y_controller = PIDController(1.5, 0, 0)
            self.heading_controller = ProfiledPIDControllerRadians(
                1.5, 0, 0, TrapezoidProfileRadians.Constraints(1, 1)
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

    def do_autodrive(self) -> None:
        self.engage()
