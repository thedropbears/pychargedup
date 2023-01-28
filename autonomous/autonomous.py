from magicbot import AutonomousStateMachine, state, timed_state, tunable
from wpimath.geometry import Pose2d, Rotation2d
from controllers.movement import Movement
from components.chassis import Chassis
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from components.vision import Vision
from dataclasses import dataclass
import math


class AutoBase:
    ...


@dataclass
class Goal:
    pose: Pose2d
    approach_dir: Rotation2d


class VisionTestAuto(AutonomousStateMachine):
    MODE_NAME = "Vision Test"
    DEFAULT = True

    chassis: Chassis
    movement: Movement
    vision: Vision

    start_pos_x = tunable(2)
    start_pos_y = tunable(2)
    start_pos_rot = tunable(0)

    def __init__(self):
        self.rotation_controller = ProfiledPIDController(
            6, 0.5, 0.5, TrapezoidProfile.Constraints(3, 5)
        )
        self.angles = [
            90,
            -90,
            45,
            -45,
            0,
        ]
        self.goal_idx = 0

    @state(first=True)
    def start(self):
        self.chassis.set_pose(
            Pose2d(self.start_pos_x, self.start_pos_y, self.start_pos_rot)
        )
        self.goal_idx = 0
        self.next_state("turn")

    @timed_state(duration=3, next_state="wait")
    def turn(self, initial_call):
        if initial_call:
            self.rotation_controller.reset(self.chassis.get_rotation().radians())
            goal = self.angles[self.goal_idx]
            print("goal:", goal)
            self.rotation_controller.setGoal(math.radians(goal))

        omega = self.rotation_controller.calculate(
            self.chassis.get_rotation().radians()
        )
        self.movement.set_input(0, 0, omega, True)
        self.vision.should_log = True

    @timed_state(duration=0.1, next_state="turn")
    def wait(self, initial_call):
        if initial_call:
            self.goal_idx += 1
            if self.goal_idx >= len(self.angles):
                self.next_state("finished")
        self.vision.should_log = True

    @state
    def finished(self):
        omega = self.rotation_controller.calculate(
            self.chassis.get_rotation().radians()
        )
        print(omega)
        self.movement.set_input(0, 0, omega, True)
