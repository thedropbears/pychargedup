from magicbot import StateMachine, state, default_state, tunable
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

from typing import Optional


class Movement(StateMachine):
    chassis: Chassis
    field: Field2d

    # When on True, a trajectory is generated every code run to be displayed
    # When on False, a trajectory is only generated when needed to save resources.
    debug_trajectory = tunable(False)

    POSITION_TOLERANCE = 0.025
    ANGLE_TOLERANCE = math.radians(2)

    def __init__(self) -> None:
        self.inputs = (0.0, 0.0, 0.0)
        self.drive_local = False

        self.goal = Pose2d(math.inf, math.inf, math.inf)
        self.waypoints: Optional[list[Translation2d]] = None
        self.is_pickup = False
        self.time_remaining = 3
        self.set_goal(
            Pose2d(1.5, 6.2, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)
        )

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")

    def generate_trajectory(self) -> Trajectory:
        """Generates a trajectory to self.goal and displays it"""
        x_controller = PIDController(2.5, 0, 0)
        y_controller = PIDController(2.5, 0, 0)
        heading_controller = ProfiledPIDControllerRadians(
            5.5, 0, 0, TrapezoidProfileRadians.Constraints(2, 2)
        )
        heading_controller.enableContinuousInput(math.pi, -math.pi)

        self.drive_controller = HolonomicDriveController(
            x_controller, y_controller, heading_controller
        )

        chassis_velocity = self.chassis.get_velocity()
        chassis_speed = math.hypot(chassis_velocity.vx, chassis_velocity.vy)
        pose = self.chassis.get_pose()

        translation = self.goal.translation() - pose.translation()
        translation_distance = translation.norm()

        # Generating a trajectory when the robot is very close to the goal is unnecesary, so this
        # return an empty trajectory that starts at the end point so the robot won't move.
        if translation_distance <= 0.01:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        if chassis_speed < 0.2:
            # If the robot is stationary, instead of accounting for the momentum of the robot,
            # this section of code will point the control vector at the goal to avoid the robot
            # taking unnecessary turns before moving towards the goal.
            kD = 0.3

            spline_start_momentum_x = translation.x * kD
            spline_start_momentum_y = translation.y * kD

        else:
            # It is more efficient to make trajectories account for the robot's current momentum so
            # the robot doesn't make a sudden stop to reverse.
            # In most cases, this will generate a smooth curve that works better than simply reversing
            # the robot.
            kvx = 3
            kvy = 3

            spline_start_momentum_x = chassis_velocity.vx * kvx
            spline_start_momentum_y = chassis_velocity.vy * kvy

        # If the robot is close to the goal but still not enough, making the robot reverse to
        # approach the control vector is unnecessary; this constant scales the derivative of
        # the goal derivative according to the translation distance.
        # The closer the robot gets to the goal, the small the derivative is.
        end_control_vec = min(10, translation_distance * 2.8)

        goal_spline = Spline3.ControlVector(
            (self.goal.X(), self.goal_approach_dir.cos() * end_control_vec),
            (self.goal.Y(), self.goal_approach_dir.sin() * end_control_vec),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        self.config.setStartVelocity(chassis_speed)

        if self.waypoints:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start_point_spline, self.waypoints, goal_spline, self.config
            )
        else:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start_point_spline, [], goal_spline, self.config
            )

        self.robot_object.setTrajectory(trajectory)
        return trajectory

    def set_goal(
        self, goal: Pose2d, approach_direction: Rotation2d, waypoints: Optional[list[Translation2d]] = None, slow_dist=0.5
    ) -> None:
        if goal == self.goal:
            return
        self.goal = goal
        self.goal_approach_dir = approach_direction

        self.config = TrajectoryConfig(maxVelocity=2, maxAcceleration=1.5)
        self.config.addConstraint(CentripetalAccelerationConstraint(2.5))
        topRight = Translation2d(self.goal.X() + slow_dist, self.goal.Y() + slow_dist)
        bottomLeft = Translation2d(self.goal.X() - slow_dist, self.goal.Y() - slow_dist)
        self.config.addConstraint(
            RectangularRegionConstraint(
                bottomLeft, topRight, MaxVelocityConstraint(0.5)
            )
        )
        self.waypoints = waypoints

    def execute_trajectory(self, trajectory: Trajectory, state_tm: float) -> None:
        target_state = trajectory.sample(
            state_tm
        )  # Grabbing the target position at the current point in time from the trajectory.

        # Calculating the speeds required to get to the target position.
        chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(),
            target_state,
            self.goal.rotation(),
        )
        self.chassis.drive_local(
            chassis_speed.vx,
            chassis_speed.vy,
            chassis_speed.omega,
        )

        self.time_remaining = trajectory.totalTime() - state_tm

    def is_at_goal(self) -> bool:
        return (
            self.goal.translation() - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal.rotation() - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE

    # will execute if no other states are executing
    @default_state
    def manualdrive(self) -> None:
        if self.debug_trajectory:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.trajectory = self.generate_trajectory()

        self.execute_trajectory(self.trajectory, state_tm)

        if self.is_at_goal():
            # self.done()
            ...
        elif self.time_remaining < 0.2:
            # TODO Tell other components to begin action
            # print("Began scoring")
            ...
        elif self.time_remaining < 1:
            # TODO Tell other components to prepare for action
            # print("Preparing for scoring")
            ...

    def set_input(self, vx: float, vy: float, vz: float, local: bool):
        """
        vx, vy: velocities in m/s
        vz: rotational velocity in rad/s
        local: drive reletive to robot frame rather than global frame
        """
        self.inputs = (vx, vy, vz)
        self.drive_local = local

    def do_autodrive(self) -> None:
        self.engage()
