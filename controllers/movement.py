from magicbot import StateMachine, state, default_state, tunable
from components.chassis import Chassis
from utilities.pathfinding import find_path, all_waypoints, get_all_corners
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from pathplannerlib import (
    PathPlanner,
    PathPoint,
    PathPlannerTrajectory,
    PathConstraints,
    controllers,
)
from wpimath.controller import (
    PIDController,
)
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
    MAX_VEL = 2.5
    MAX_ACCEL = 2
    MAX_CENTR_ACCEL = 2.5

    # spacing of additional waypoints added to mimic the wpilib region constraint
    SLOW_DIST_SPACING = 0.1
    SLOW_VEL = 0.5

    SHOW_PATHFINDING_DEBUG = True

    def __init__(self) -> None:
        self.inputs = (0.0, 0.0, 0.0)
        self.drive_local = False

        self.goal = Pose2d(math.inf, math.inf, math.inf)
        self.goal_approach_dir = Rotation2d()
        self.waypoints: tuple[Translation2d, ...] = ()
        self.is_pickup = False
        self.time_to_goal = 3
        self.slow_dist = 0

    def setup(self):
        self.robot_object = self.field.getObject("auto_trajectory")
        self.set_goal(
            Pose2d(1.5, 6.2, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)
        )
        self.path_object = self.field.getObject("path")
        if self.SHOW_PATHFINDING_DEBUG:
            self.waypoints_object = self.field.getObject("waypoints")
            self.waypoints_object.setPoses(all_waypoints)
            self.obsticles_object = self.field.getObject("obsticles")
            self.obsticles_object.setPoses(get_all_corners())

    def generate_trajectory(self) -> PathPlannerTrajectory:
        """Generates a trajectory to self.goal and displays it"""
        pose = self.chassis.get_pose()
        distance = pose.translation().distance(self.goal.translation())
        waypoints = find_path(pose, self.goal)
        # remove first and last nodes in path
        if self.SHOW_PATHFINDING_DEBUG:
            self.path_object.setPoses(waypoints)

        chassis_velocity = self.chassis.get_velocity()
        chassis_speed = math.hypot(chassis_velocity.vx, chassis_velocity.vy)
        points: list[PathPoint] = []

        # Add starting pathpoint
        if chassis_speed < 0.1:
            # if we're not moving control vector points towards next waypoint
            dir_to_next = (waypoints[1].translation() - pose.translation()).angle()
            points.append(
                PathPoint(
                    pose.translation(), dir_to_next, pose.rotation(), chassis_speed
                )
            )
        else:
            # otherwise control vector goes with current momentum
            points.append(PathPoint.fromCurrentHolonomicState(pose, chassis_velocity))
            points[0].withNextControlLength(chassis_speed * 0.5)

        # Add waypoints from pathfinding
        for last, cur, next in zip(waypoints, waypoints[1:], waypoints[2:]):
            dir_to_next = (next.translation() - cur.translation()).angle()
            dir_from_prev = (cur.translation() - last.translation()).angle()
            points.append(PathPoint(cur.translation(), dir_to_next, cur.rotation()))
            length = max(math.cos((dir_to_next - dir_from_prev).radians()) + 0.1, 0.1)
            points[-1].withPrevControlLength(length)

        # Add waypoints near goal with low speed so the approach is slow
        approach_dist = min(self.slow_dist, distance)
        slow_dist_transform = Translation2d(self.SLOW_DIST_SPACING, 0).rotateBy(
            self.goal_approach_dir
        )
        for i in range(1 + math.ceil(approach_dist / self.SLOW_DIST_SPACING), 0, -1):
            pos = self.goal.translation() - slow_dist_transform * i
            points.append(
                PathPoint(
                    pos, self.goal_approach_dir, self.goal.rotation(), self.SLOW_VEL
                )
            )
        # Add goal waypoint
        points.append(
            PathPoint(
                self.goal.translation(), self.goal_approach_dir, self.goal.rotation()
            )
        )

        constraints = PathConstraints(self.MAX_VEL, self.MAX_ACCEL)
        trajectory = PathPlanner.generatePath(constraints, points)

        x_controller = PIDController(3.5, 0, 0)
        y_controller = PIDController(3.5, 0, 0)
        heading_controller = PIDController(3.0, 0, 0)
        self.drive_controller = controllers.PPHolonomicDriveController(
            x_controller, y_controller, heading_controller
        )

        self.robot_object.setTrajectory(trajectory.asWPILibTrajectory())
        return trajectory

    def set_goal(
        self,
        goal: Pose2d,
        approach_direction: Rotation2d,
        slow_dist=0.5,
    ) -> None:
        if goal == self.goal and approach_direction == self.goal_approach_dir:
            return
        self.goal = goal
        self.goal_approach_dir = approach_direction
        self.slow_dist = slow_dist

        if self.current_state == "autodrive":
            # to reset the state_tm and regen trajectory
            self.trajectory = self.generate_trajectory()
            self.time_to_goal = self.trajectory.getTotalTime()
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

        target_state = self.trajectory.sample(state_tm)

        # Calculating the speeds required to get to the target position.
        chassis_speed = self.drive_controller.calculate(
            self.chassis.get_pose(),
            target_state,
        )
        self.chassis.drive_local(
            chassis_speed.vx,
            chassis_speed.vy,
            chassis_speed.omega,
        )

        self.time_to_goal = self.trajectory.getTotalTime() - state_tm

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
