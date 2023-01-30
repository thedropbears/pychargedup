#!/usr/bin/env python3

import wpilib
import magicbot

from controllers.movement import Movement
from components.intake import Intake
from components.chassis import Chassis
from components.vision import Vision
from components.arm import Arm
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    movement: Movement

    # Components
    chassis: Chassis
    vision: Vision
    arm: Arm
    intake: Intake

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def teleopPeriodic(self) -> None:
        drop_off = self.gamepad.getAButton()
        spin_rate = 6
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * Chassis.max_wheel_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * Chassis.max_wheel_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.gamepad.getXButton()

        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)

        if drop_off:
            self.movement.do_trajectory()

    def testInit(self) -> None:
        self.arm.on_enable()

    def testPeriodic(self) -> None:
        right_trigger = self.gamepad.getRightTriggerAxis()
        left_trigger = self.gamepad.getLeftTriggerAxis()
        self.arm.rotation_motor.set((left_trigger - right_trigger) * 0.5)
        # self.arm._rotation_motor_right.set((right_trigger - right_trigger) * 0.5)
        # self.arm.extension_motor.set(left_trigger * 0.1)
        # self.arm.set_angle(self.arm.goal_angle + right_trigger * 0.02)
        # self.arm.extension_motor.set(left_trigger * 0.1)
        # self.arm.set_length(self.arm.goal_extension + (left_trigger - right_trigger) * 0.02 * 2)

        # self.arm.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
