#!/usr/bin/env python3

import wpilib
import magicbot

from controllers.movement import Movement
from controllers.leds import LedController
from components.chassis import Chassis
from components.vision import Vision
from components.leds import StatusLights
from wpimath.geometry import Pose2d, Rotation2d
import math
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    movement: Movement
    led_controller: LedController

    # Components
    chassis: Chassis
    vision: Vision
    status_lights: StatusLights

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.leds = wpilib.AddressableLED(2)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # if self.isReal():
        #     try:
        #         from cscore import CameraServer  # type: ignore
        #     except ImportError:
        #         self.logger.exception("Could not import CameraServer")
        #     else:
        #         CameraServer.startAutomaticCapture()

    def teleopPeriodic(self) -> None:
        drop_off = self.gamepad.getAButton()
        pick_up = self.gamepad.getXButton()
        spin_rate = 6
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * Chassis.max_wheel_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * Chassis.max_wheel_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()

        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)
        if drop_off:
            self.movement.do_autodrive(Pose2d(3, 0, 0), Rotation2d(0))
        elif pick_up:
            self.movement.do_autodrive(Pose2d(0, -1, math.pi), Rotation2d(math.pi))


if __name__ == "__main__":
    wpilib.run(MyRobot)
