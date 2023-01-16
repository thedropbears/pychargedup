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

    def __init__(self) -> None:
        self.inputs = (0.0, 0.0, 0.0)
        self.drive_local = False
        # Initialise the goal pose, control vector and rotation2d
        # These will be used until set_goal() is called, which will then define new
        # variables according to the provided parameters.
        self.goal = Pose2d(3, 0, 0)
        self.goal_spline = Spline3.ControlVector(
            (self.goal.X(), -6),
            (
                self.goal.Y(),
                0,
            ),  # This will be recalculated every code run to adjust derivatives accordingly.
        )
        self.goal_rotation = Rotation2d()

        # This vairable controls if trajectory generation will be set up for debugging
        # When on True, a trajectory is genearted every code run and will consistently
        # be painted onto Glass' SmartDashboard.
        # When on False, a trajectory is only generated when needed to save robotRIO resources.
        self.debug_trajectory = tunable(False)

        self.set_goal(Pose2d(3, 0, 0), Rotation2d(0))

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")

    def generate_trajectory(self) -> Trajectory:
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

            self.spline_start_momentum_x = translation.x * kD
            self.spline_start_momentum_y = translation.y * kD

        else:
            # It is more efficient to make trajectories account for the robot's current momentum so
            # the robot doesn't make a sudden stop to reverse.
            # In most cases, this will generate a smooth curve that works better than simply reversing
            # the robot.
            kvx = 3
            kvy = 3

            self.spline_start_momentum_x = chassis_velocity.vx * kvx
            self.spline_start_momentum_y = chassis_velocity.vy * kvy

        # If the robot is close to the goal but still not enough, making the robot reverse to
        # approach the control vector is unnecessary; this constant scales the derivative of
        # the goal derivative according to the translation distance.
        # The closer the robot gets to the goal, the small the derivative is.
        k_spline = min(8, translation_distance * 2)

        self.goal_spline = Spline3.ControlVector(
            (
                self.goal.X(),
                self.goal_rotation.cos() * k_spline,
            ),  # Actually scaling the control vector using the above constant.
            (self.goal.Y(), self.goal_rotation.sin() * k_spline),
        )

        start_point_spline = Spline3.ControlVector(
            (pose.x, self.spline_start_momentum_x), (pose.y, self.spline_start_momentum_y)
        )

        self.config.setStartVelocity(math.hypot(y_velocity, x_velocity))

        self.auto_trajectory = TrajectoryGenerator.generateTrajectory(
            start_point_spline, [], self.goal_spline, self.config
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
        if self.debug_trajectory is True:
            self.generate_trajectory()
        if self.drive_local:
            self.chassis.drive_local(*self.inputs)
        else:
            self.chassis.drive_field(*self.inputs)

    @state(first=True)
    def autodrive(self, state_tm: float, initial_call: bool) -> None:
        if initial_call:
            # When the robot is following a trajectory, it doesn't need to re-define the
            # PID controllers nor does it need to generate another trajectory.
            # Therefore, this section of code only runs when the state is first active.
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

        target_state = self.trajectory.sample(
            state_tm
        )  # Grabbing the target position at the current point in time from the trajectory.

        chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(),
            target_state,
            self.goal.rotation(),  # Calculating the speeds required to get to the target position.
        )
        self.chassis.drive_local(
            chassis_speed.vx,
            chassis_speed.vy,
            chassis_speed.omega,
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
        # Sets teleoperated drive inputs.
        self.inputs = (vx, vy, vz)
        self.drive_local = local

    def do_autodrive(self, goal: Pose2d, approach_direction: Rotation2d) -> None:
        # Sets the goal and execute the autodrive state.
        if goal == self.goal:
            self.set_goal(goal, approach_direction)
        self.engage()
