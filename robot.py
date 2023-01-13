#!/usr/bin/env python3

import wpilib
import magicbot

from controllers.movement import Movement
from components.chassis import Chassis
from components.vision import Vision

from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    movement: Movement

    # Components
    chassis: Chassis
    vision: Vision

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        if self.isReal():
            try:
                from cscore import CameraServer  # type: ignore
            except ImportError:
                self.logger.exception("Could not import CameraServer")
            else:
                CameraServer.startAutomaticCapture()

    def teleopPeriodic(self) -> None:
        auto_driving = self.gamepad.getAButton()
        spin_rate = 6
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * Chassis.max_wheel_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * Chassis.max_wheel_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()

        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)
        if auto_driving:
            self.movement.do_autodrive()


if __name__ == "__main__":
    wpilib.run(MyRobot)
