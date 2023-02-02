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

    def teleopInit(self) -> None:
        self.vision.add_to_estimator = True

    def teleopPeriodic(self) -> None:
        autodrive = self.gamepad.getAButton()
        spin_rate = 6
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * Chassis.max_wheel_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * Chassis.max_wheel_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.gamepad.getXButton()

        if self.gamepad.getYButtonPressed():
            self.intake.end_intake()

        if self.gamepad.getXButtonPressed():
            self.intake.do_intake()

        if self.gamepad.getLeftBumper():
            self.intake.end_intake()

        if self.gamepad.getRightBumper():
            self.intake.do_intake()

        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)
        if autodrive:
            self.movement.do_autodrive()


if __name__ == "__main__":
    wpilib.run(MyRobot)
