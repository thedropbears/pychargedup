from magicbot import (
    StateMachine,
    state,
    default_state,
    tunable,
    will_reset_to,
)
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
from utilities.game import is_red

from utilities.functions import clamp

import wpiutil.log


class Movement(StateMachine):
    chassis: Chassis
    field: Field2d
    control_loop_wait_time: float

    # When on True, a trajectory is generated every code run to be displayed
    # When on False, a trajectory is only generated when needed to save resources.
    debug_trajectory = tunable(False)

    POSITION_TOLERANCE = 0.025
    END_CONTROL_SCALER = 1
    ANGLE_TOLERANCE = math.radians(2)
    driver_inputs = will_reset_to((0.0, 0.0, 0.0))
    inputs_lock = will_reset_to(False)

    BALANCE_MAX_SPEED = 0.5
    BALANCE_GAIN = tunable(1.0)
    BALANCE_RATE_GAIN = tunable(
        -0.2
    )  # Needs to be negative to counteract increasing pitch when the charge station shifts
    BALANCE_TILT_ANGLE_THRESHOLD = math.radians(2)
    BALANCE_TILT_RATE_THRESHOLD = math.radians(2)

    data_log: wpiutil.log.DataLog

    def __init__(self) -> None:
        self.drive_local = False

        self.goal = Pose2d(math.inf, math.inf, math.inf)
        self.goal_approach_dir = Rotation2d()
        self.config = TrajectoryConfig(maxVelocity=2, maxAcceleration=1.5)
        self.waypoints: tuple[Translation2d, ...] = ()
        self.is_pickup = False
        self.time_to_goal = 3

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")
        self.goal_entry = wpiutil.log.DoubleArrayLogEntry(
            self.data_log, "movement_goal"
        )
        self.waypoints_entry = wpiutil.log.DoubleArrayLogEntry(
            self.data_log, "movement_waypoints"
        )
        self.set_goal(
            Pose2d(1.5, 6.2, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)
        )

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

        next_pos = self.waypoints[0] if self.waypoints else self.goal.translation()
        translation = next_pos - pose.translation()

        # Generating a trajectory when the robot is very close to the goal is unnecesary, so this
        # return an empty trajectory that starts at the end point so the robot won't move.
        distance_to_goal = (self.goal.translation() - pose.translation()).norm()
        if distance_to_goal <= 0.01:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        if chassis_speed < 0.5:
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
            kvx = 1.5
            kvy = 1.5

            spline_start_momentum_x = chassis_velocity.vx * kvx
            spline_start_momentum_y = chassis_velocity.vy * kvy

        # If the robot is close to the goal but still not enough, making the robot reverse to
        # approach the control vector is unnecessary; this constant scales the derivative of
        # the goal derivative according to the translation distance.
        # The closer the robot gets to the goal, the small the derivative is.
        end_control_length = 1

        goal_spline = Spline3.ControlVector(
            (self.goal.X(), self.goal_approach_dir.cos() * end_control_length),
            (self.goal.Y(), self.goal_approach_dir.sin() * end_control_length),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, spline_start_momentum_x),
            (pose.y, spline_start_momentum_y),
        )

        self.config.setStartVelocity(chassis_speed)
        try:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start_point_spline, list(self.waypoints), goal_spline, self.config
            )
        except RuntimeError:
            return Trajectory([Trajectory.State(0, 0, 0, pose)])

        self.robot_object.setTrajectory(trajectory)
        return trajectory

    def set_goal(
        self,
        goal: Pose2d,
        approach_direction: Rotation2d,
        waypoints: tuple[Translation2d, ...] = (),
        slow_dist: float = 0.5,
        max_accel: float = 0.5,
        max_vel: float = 1,
    ) -> None:
        if (
            goal == self.goal
            and approach_direction == self.goal_approach_dir
            and waypoints == self.waypoints
        ):
            return
        self.goal_entry.append(
            [self.goal.x, self.goal.y, float(self.goal.rotation().radians())]
        )
        for w in waypoints:
            self.waypoints_entry.append([w.x, w.y])
        self.goal = goal
        self.goal_approach_dir = approach_direction

        self.config = TrajectoryConfig(maxVelocity=max_vel, maxAcceleration=max_accel)
        self.config.addConstraint(CentripetalAccelerationConstraint(5.0))
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
        real_at_goal = (
            self.goal.translation() - self.chassis.get_pose().translation()
        ).norm() < self.POSITION_TOLERANCE and abs(
            (self.goal.rotation() - self.chassis.get_rotation()).radians()
        ) < self.ANGLE_TOLERANCE
        hitting_wall = self.time_to_goal < -0.1 and self.chassis.may_be_stalled()
        return real_at_goal or hitting_wall

    # will execute if no other states are executing
    @default_state
    def manualdrive(self) -> None:
        if self.debug_trajectory:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.driver_inputs)
        else:
            self.chassis.drive_field(*self.driver_inputs)

    @state(first=True)
    def autodrive(self, state_tm: float, initial_call: bool) -> None:
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

    @state(must_finish=True)
    def balance(self):
        tilt_error = 0.0 - self.chassis.get_tilt()
        tilt_rate_error = 0.0 - self.chassis.get_tilt_rate()
        speed_x = (
            tilt_error * self.BALANCE_GAIN + tilt_rate_error * self.BALANCE_RATE_GAIN
        )
        speed_x = clamp(
            speed_x, -Movement.BALANCE_MAX_SPEED, Movement.BALANCE_MAX_SPEED
        )
        self.chassis.drive_local(speed_x, 0, 0)
        if (
            abs(tilt_error) < Movement.BALANCE_TILT_ANGLE_THRESHOLD
            and abs(tilt_rate_error) < Movement.BALANCE_TILT_RATE_THRESHOLD
        ):
            self.done()

    def set_input(
        self, vx: float, vy: float, vz: float, local: bool, override: bool = False
    ) -> None:
        """
        vx, vy: velocities in m/s
        vz: rotational velocity in rad/s
        local: drive reletive to robot frame rather than global frame
        override: override driver lockout
        """
        if not self.inputs_lock or override:
            if is_red():
                vx = -vx
                vy = -vy

            self.driver_inputs = (vx, vy, vz)
            self.drive_local = local

    def do_autodrive(self) -> None:
        self.engage("autodrive")

    # for testing
    def toggle_balance(self) -> None:
        if self.is_executing:
            self.start_balance()
        else:
            self.stop_balance()

    def start_balance(self) -> None:
        self.engage("balance", force=True)

    def stop_balance(self) -> None:
        self.done()
