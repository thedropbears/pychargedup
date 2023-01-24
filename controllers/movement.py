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


class Movement(StateMachine):
    chassis: Chassis
    field: Field2d

    # This vairable controls if trajectory generation will be set up for debugging
    # When on True, a trajectory is genearted every code run and will consistently
    # be painted onto Glass' SmartDashboard.
    # When on False, a trajectory is only generated when needed to save robotRIO resources.
    debug_trajectory = tunable(False)

    POSITION_TOLERANCE = 0.025
    ANGLE_TOLERANCE = math.radians(2)

    def __init__(self) -> None:
        self.inputs = (0.0, 0.0, 0.0)
        self.drive_local = False

        self.goal = Pose2d(math.inf, math.inf, math.inf)
        self.is_pickup = False
        self.time_remaining = 3
        self.set_goal(Pose2d(3, 0, 0), Rotation2d(0))

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")

    def generate_trajectory(self) -> Trajectory:
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
        pose = self.chassis.get_pose()
        x_velocity = chassis_velocity.vx
        y_velocity = chassis_velocity.vy

        translation = self.goal.translation() - pose.translation()
        translation_distance = translation.norm()

        # Generating a trajectory when the robot is very close to the goal is unnecesary, so this
        # return an empty trajectory that starts at the end point so the robot won't move.
        if translation_distance <= 0.01:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        if math.hypot(x_velocity, y_velocity) < 0.2:
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
        k_spline = min(12, translation_distance * 2.8)

        self.goal_spline = Spline3.ControlVector(
            (
                self.goal.X(),
                self.goal_rotation.cos() * k_spline,
            ),  # Actually scaling the control vector using the above constant.
            (self.goal.Y(), self.goal_rotation.sin() * k_spline),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        self.config.setStartVelocity(math.hypot(y_velocity, x_velocity))

        self.auto_trajectory = TrajectoryGenerator.generateTrajectory(
            start_point_spline, [], self.goal_spline, self.config
        )

        self.robot_object.setTrajectory(self.auto_trajectory)
        return self.auto_trajectory

    def set_goal(self, goal: Pose2d, approach_direction: Rotation2d) -> None:
        if goal != self.goal:
            self.goal = goal
            self.goal_rotation = approach_direction

            self.config = TrajectoryConfig(2, 1.5)
            self.config.addConstraint(CentripetalAccelerationConstraint(1))
            topRight = Translation2d(self.goal.X() + 0.25, self.goal.Y() + 0.25)
            bottomLeft = Translation2d(self.goal.X() - 0.25, self.goal.Y() - 0.25)
            self.config.addConstraint(
                RectangularRegionConstraint(
                    bottomLeft, topRight, MaxVelocityConstraint(0.5)
                )
            )

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

        self.time_remaining = self.auto_trajectory.totalTime() - state_tm

    def is_at_goal(self) -> bool:
        return (
            self.goal.translation() - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal.rotation() - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE

    # will execute if no other states are executing
    @default_state
    def manualdrive(self) -> None:
        if self.debug_trajectory is True:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state
    def score(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.set_goal(Pose2d(0.9, 0.9, 0), Rotation2d.fromDegrees(180))
            self.calc_trajectory = self.generate_trajectory()

        if self.is_at_goal():
            self.next_state("arrived_at_score")
            return
        elif self.is_pickup:
            self.next_state("pickup")

        if self.time_remaining < 2 and self.time_remaining > 0.5:
            # TODO Tell other components to prepare for scoring
            print("Preparing for scoring")
            pass
        elif self.time_remaining <= 0.5:
            # TODO Tell other components to begin scoring
            print("Began scoring")
            pass

        self.execute_trajectory(self.calc_trajectory, state_tm)

    @state(first=True)
    def pickup(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            self.set_goal(Pose2d(1.6, 2, math.pi), Rotation2d(math.pi))
            self.calc_trajectory = self.generate_trajectory()

        if self.is_at_goal():
            self.next_state("arrived_at_pickup")
            return
        elif not self.is_pickup:
            self.next_state("score")
            return

        if self.time_remaining < 2 and self.time_remaining > 0.5:
            # TODO Tell other components to prepare for pickup
            print("Preparing for pickup")
            pass
        elif self.time_remaining <= 0.5:
            # TODO Tell other components to begin pickup
            print("Began pickup")
            pass

        self.execute_trajectory(self.calc_trajectory, state_tm)

    @state
    def arrived_at_pickup(self, state_tm: float, initial_call: bool) -> None:
        print("Arrived at pickup")
        if initial_call:
            self.is_pickup = False
        if state_tm > 0.2:
            self.next_state("score")

    @state
    def arrived_at_score(self, state_tm: float, initial_call: bool) -> None:
        print("Arrived at score")
        if initial_call:
            self.is_pickup = False
        if state_tm > 0.2:
            self.next_state("pickup")

    def set_input(self, vx: float, vy: float, vz: float, local: bool):
        # Sets teleoperated drive inputs.
        self.inputs = (vx, vy, vz)
        self.drive_local = local

    def do_trajectory(self) -> None:
        self.engage()
