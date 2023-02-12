#!/usr/bin/env python3

import wpilib
import magicbot

from controllers.movement import Movement
from controllers.scoring import ScoringController
from components.intake import Intake
from components.chassis import Chassis
from components.vision import Vision
from components.arm import Arm, Setpoints
from components.gripper import Gripper
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    scoring: ScoringController
    movement: Movement

    # Components
    chassis: Chassis
    vision: Vision
    arm: Arm
    intake: Intake
    gripper: Gripper

    max_speed = magicbot.tunable(Chassis.max_wheel_speed * 0.95)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def teleopInit(self) -> None:
        self.vision.add_to_estimator = True

    def teleopPeriodic(self) -> None:
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * self.max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * self.max_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)

        # Automation
        right_trigger = self.gamepad.getRightTriggerAxis() > 0.3
        left_trigger = self.gamepad.getLeftTriggerAxis() > 0.3
        if right_trigger:
            self.scoring.cone_pickup_side_right = True
        if left_trigger:
            self.scoring.cone_pickup_side_right = False
        self.scoring.autodrive = right_trigger or left_trigger

        # Intake
        if self.gamepad.getRightBumperPressed():
            self.scoring.wants_to_intake = True
        if self.gamepad.getLeftBumperPressed():
            self.scoring.wants_to_intake = False

        self.scoring.engage()

        # Manual overrides
        # Claw
        if self.gamepad.getYButton():
            self.gripper.wants_to_close = not self.gripper.wants_to_close
        if self.gamepad.getXButton():
            self.gripper.open()
        if self.gamepad.getBButton():
            self.gripper.close()

        # Arm
        dpad_angle = self.gamepad.getPOV()
        # up
        if dpad_angle == 0:
            self.arm.goto_setpoint(Setpoints.SCORE_CONE_HIGH)
        # right
        elif dpad_angle == 90:
            self.arm.goto_setpoint(Setpoints.SCORE_CONE_MID)
        # down
        elif dpad_angle == 180:
            self.arm.goto_setpoint(Setpoints.HANDOFF)
        # left
        elif dpad_angle == 270:
            self.arm.goto_setpoint(Setpoints.PICKUP_CONE)

    def testInit(self) -> None:
        self.arm.on_enable()
        self.vision.add_to_estimator = False

    def testPeriodic(self) -> None:
        right_trigger = self.gamepad.getRightTriggerAxis()
        left_trigger = self.gamepad.getLeftTriggerAxis()
        self.arm.rotation_motor.set(right_trigger * 0.5)
        self.arm._rotation_motor_follower.set(left_trigger * 0.5)
        # self.arm.extension_motor.set(left_trigger * 0.1)
        # self.arm.set_angle(self.arm.goal_angle + right_trigger * 0.02)
        # self.arm.extension_motor.set(left_trigger * 0.1)
        # self.arm.set_length(self.arm.goal_extension + (left_trigger - right_trigger) * 0.02 * 2)

        # self.arm.execute()

        self.vision.execute()

    def disabledInit(self) -> None:
        self.vision.add_to_estimator = False

    def disabledPeriodic(self):
        self.vision.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
