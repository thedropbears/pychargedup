#!/usr/bin/env python3

import wpilib
import wpilib.event
import magicbot
from wpimath.geometry import Quaternion, Rotation3d, Translation3d

from controllers.movement import Movement
from controllers.scoring import ScoringController


from controllers.acquire_cone import AcquireConeController
from controllers.acquire_cube import AcquireCubeController
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController

from components.intake import Intake
from components.chassis import Chassis
from components.vision import VisualLocalizer
from components.arm import Arm, Setpoints
from components.gripper import Gripper
from components.leds import StatusLights
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    scoring: ScoringController
    movement: Movement

    acquire_cone: AcquireConeController
    acquire_cube: AcquireCubeController
    recover: RecoverController
    score_game_piece: ScoreGamePieceController

    # arm_controller: ArmController

    # Components
    chassis: Chassis
    arm: Arm
    intake: Intake
    status_lights: StatusLights
    gripper: Gripper

    port_localizer: VisualLocalizer
    starboard_localizer: VisualLocalizer

    max_speed = magicbot.tunable(Chassis.max_wheel_speed * 0.95)

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        self.gamepad = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        self.event_loop = wpilib.event.EventLoop()
        self.right_trigger_down = self.gamepad.rightTrigger(
            0.3, self.event_loop
        ).rising()
        self.right_trigger_up = self.gamepad.rightTrigger(
            0.3, self.event_loop
        ).falling()
        self.left_trigger_down = self.gamepad.leftTrigger(0.3, self.event_loop).rising()
        self.left_trigger_up = self.gamepad.leftTrigger(0.3, self.event_loop).falling()
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.port_localizer_name = "cam_port"
        self.port_localizer_pos = Translation3d(-0.35001, 0.06583, 0.25)
        self.port_localizer_rot = Rotation3d(
            Quaternion(
                0.0850897952914238,
                0.21561633050441742,
                -0.9725809097290039,
                0.018864024430513382,
            )
        )

        self.starboard_localizer_name = "cam_starboard"
        self.starboard_localizer_pos = Translation3d(-0.35001, -0.06583, 0.247)
        self.starboard_localizer_rot = Rotation3d(
            Quaternion(
                0.08508981764316559,
                -0.21561576426029205,
                -0.9725810289382935,
                -0.01886390522122383,
            )
        )

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.event_loop.poll()
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * self.max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * self.max_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)

        # Cone Pickup
        if self.right_trigger_down.getAsBoolean():
            self.acquire_cone.target_right()
            self.acquire_cone.engage("deploying_arm")
        if self.left_trigger_down.getAsBoolean():
            self.acquire_cone.target_left()
            self.acquire_cone.engage("deploying_arm")

        if self.left_trigger_up.getAsBoolean() or self.right_trigger_up.getAsBoolean():
            self.acquire_cone.done()

        # Intake
        if self.gamepad.getRightBumperPressed():
            self.acquire_cube.engage()
        if self.gamepad.getLeftBumperPressed():
            self.acquire_cube.done()

        # Scoring
        if self.gamepad.getAButtonPressed():
            self.score_game_piece.prefer_high()
            self.score_game_piece.engage("deploying_arm")
        if self.gamepad.getBButtonPressed():
            self.score_game_piece.prefer_mid()
            self.score_game_piece.engage("deploying_arm")

        if self.gamepad.getAButtonReleased() or self.gamepad.getBButtonReleased():
            self.score_game_piece.done()

        # Manual overrides
        # Claw
        if self.gamepad.getStartButtonPressed():
            self.gripper.close()
        if self.gamepad.getBackButtonPressed():
            self.gripper.open()

        # Arm
        dpad_angle = self.gamepad.getPOV()
        # up
        if dpad_angle == 0:
            self.arm.go_to_setpoint(Setpoints.SCORE_CONE_HIGH)
        # right
        elif dpad_angle == 90:
            self.arm.go_to_setpoint(Setpoints.FORWARDS)
        # down
        elif dpad_angle == 180:
            self.gripper.open()
            self.arm.go_to_setpoint(Setpoints.HANDOFF)
        # left
        elif dpad_angle == 270:
            self.arm.go_to_setpoint(Setpoints.SCORE_CONE_MID)

    def testInit(self) -> None:
        self.arm.on_enable()
        self.arm.stop()
        self.arm.homing = False

    def testPeriodic(self) -> None:
        dpad_angle = self.gamepad.getPOV()

        # Intake
        if self.gamepad.getYButton():
            if dpad_angle == 0:
                self.intake.deploy()
            if dpad_angle == 180:
                self.intake.retract()

        # Claw
        if self.gamepad.getBButton():
            if dpad_angle == 0:
                self.gripper.close()
            if dpad_angle == 180:
                self.gripper.open()

        # Arm
        if self.gamepad.getAButton():
            # TODO add functionality here if required
            pass

        # State machines
        if self.gamepad.getXButton():
            if self.gamepad.getLeftBumperPressed():
                self.acquire_cone.target_left()
            if self.gamepad.getRightBumperPressed():
                self.acquire_cone.target_right()
            if dpad_angle == 180:
                self.acquire_cone.engage()
            if dpad_angle == 0:
                self.acquire_cube.engage()
            if dpad_angle == 90 or dpad_angle == 270:
                self.recover.engage()
            if self.gamepad.getLeftStickButtonPressed():
                self.score_game_piece.engage()

        # Cancel any running controllers
        if self.gamepad.getBackButtonPressed():
            self.cancel_controllers()

        # self.scoring.execute()
        self.arm.execute()
        self.intake.execute()
        self.gripper.execute()
        self.port_localizer.execute()
        self.starboard_localizer.execute()
        self.status_lights.execute()

        # Tick the controllers
        # These will only do anything if engage() has been called on them
        self.acquire_cone.execute()
        self.acquire_cube.execute()
        self.score_game_piece.execute()
        self.recover.execute()

    def cancel_controllers(self):
        self.acquire_cone.done()
        self.acquire_cube.done()
        self.score_game_piece.done()
        self.recover.engage()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self):
        self.port_localizer.execute()
        self.starboard_localizer.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
