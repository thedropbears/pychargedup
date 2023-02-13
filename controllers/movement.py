from magicbot import StateMachine, state, default_state, tunable
from components.chassis import Chassis
from utilities.functions import clamp
from utilities.pathfinding import find_path, to_poses
from utilities.pathfinding import waypoints as pathfinding_waypoints
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

    # When on True, a trajectory is generated every code run to be displayed
    # When on False, a trajectory is only generated when needed to save resources.
    debug_trajectory = tunable(True)

    POSITION_TOLERANCE = 0.025
    ANGLE_TOLERANCE = math.radians(2)
    # constraints for path following
    MAX_VEL = 3.0
    MAX_ACCEL = 2
    MAX_CENTR_ACCEL = 2.5

    def __init__(self) -> None:
        self.inputs = (0.0, 0.0, 0.0)
        self.drive_local = False

        self.goal = Pose2d(math.inf, math.inf, math.inf)
        self.goal_approach_dir = Rotation2d()
        self.config = TrajectoryConfig(maxVelocity=2, maxAcceleration=1.5)
        self.waypoints: tuple[Translation2d, ...] = ()
        self.is_pickup = False
        self.time_to_goal = 3

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")
        self.set_goal(
            Pose2d(1.5, 6.2, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)
        )
        self.path_object = self.field.getObject("path")
        self.waypoints_object = self.field.getObject("waypoints")
        waypoints_as_poses = [Pose2d(p[0], p[1], 0) for p in pathfinding_waypoints]
        self.waypoints_object.setPoses(waypoints_as_poses)

    def generate_trajectory(self) -> Trajectory:
        """Generates a trajectory to self.goal and displays it"""
        pose = self.chassis.get_pose()
        waypoints = find_path(pose.translation(), self.goal.translation())
        # remove first and last nodes in path
        waypoints.pop()
        waypoints.pop(0)
        self.path_object.setPoses(to_poses(waypoints))

        chassis_velocity = self.chassis.get_velocity()
        chassis_speed = math.hypot(chassis_velocity.vx, chassis_velocity.vy)

        if len(waypoints):
            translation = waypoints[0] - pose.translation()
            translation_distance = translation.norm()
            end_control_scale = min(10, translation_distance * 0.5)
        else:
            translation = self.goal.translation() - pose.translation()
            translation_distance = translation.norm()
            end_control_scale = min(10, translation_distance * 2)

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

        goal_spline = Spline3.ControlVector(
            (self.goal.X(), self.goal_approach_dir.cos() * end_control_scale),
            (self.goal.Y(), self.goal_approach_dir.sin() * end_control_scale),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        self.config.setStartVelocity(chassis_speed)
        trajectory = TrajectoryGenerator.generateTrajectory(
            start_point_spline, waypoints, goal_spline, self.config
        )
        rotation_needed = (pose.rotation() - self.goal.rotation()).radians()
        if rotation_needed == 0:
            rotate_rate = 2
        else:
            rotate_rate = clamp(
                2 * abs(rotation_needed) / trajectory.totalTime(), 0.1, 10
            )
        x_controller = PIDController(2.5, 0, 0)
        y_controller = PIDController(2.5, 0, 0)
        heading_controller = ProfiledPIDControllerRadians(
            5.5, 0, 0, TrapezoidProfileRadians.Constraints(rotate_rate, 3)
        )
        heading_controller.enableContinuousInput(math.pi, -math.pi)
        self.drive_controller = HolonomicDriveController(
            x_controller, y_controller, heading_controller
        )

        self.robot_object.setTrajectory(trajectory)
        return trajectory

    def set_goal(
        self,
        goal: Pose2d,
        approach_direction: Rotation2d,
        waypoints: tuple[Translation2d, ...] = (),
        slow_dist=0.5,
    ) -> None:
        if (
            goal == self.goal
            and approach_direction == self.goal_approach_dir
            and waypoints == self.waypoints
        ):
            return
        self.goal = goal
        self.goal_approach_dir = approach_direction

        self.config = TrajectoryConfig(
            maxVelocity=self.MAX_VEL, maxAcceleration=self.MAX_ACCEL
        )
        self.config.addConstraint(
            CentripetalAccelerationConstraint(self.MAX_CENTR_ACCEL)
        )
        topRight = Translation2d(self.goal.X() + slow_dist, self.goal.Y() + slow_dist)
        bottomLeft = Translation2d(self.goal.X() - slow_dist, self.goal.Y() - slow_dist)
        self.config.addConstraint(
            RectangularRegionConstraint(
                bottomLeft, topRight, MaxVelocityConstraint(0.5)
            )
        )
        self.waypoints = waypoints
        if self.current_state == "autodrive":
            # to reset the state_tm and regen trajectory
            self.trajectory = self.generate_trajectory()
            self.time_to_goal = self.trajectory.totalTime()
            self.next_state("autodrive")

    def is_at_goal(self) -> bool:
        return (
            self.goal.translation() - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal.rotation() - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE

    # will execute if no other states are executing
    @default_state
    def manualdrive(self) -> None:
        self.chassis.do_fudge = True
        if self.debug_trajectory:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self, state_tm: float, initial_call: bool) -> None:
        self.chassis.do_fudge = False
        if initial_call:
            self.trajectory = self.generate_trajectory()

        target_state = self.trajectory.sample(
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

        self.time_to_goal = self.trajectory.totalTime() - state_tm

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
