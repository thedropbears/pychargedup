#!/usr/bin/env python3

import wpilib
import magicbot
from wpimath.geometry import Quaternion, Rotation3d, Translation3d

from controllers.movement import Movement
from controllers.scoring import ScoringController

from controllers.arm import ArmController

from controllers.acquire_cone import AcquireConeController
from controllers.acquire_cube import AcquireCubeController
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController

from components.intake import Intake
from components.chassis import Chassis
from components.vision import VisualLocalizer
from components.arm import Arm, Setpoints
from components.gripper import Gripper
from components.leds import StatusLights, DisplayType, LedColors
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    scoring: ScoringController
    movement: Movement

    acquire_cone: AcquireConeController
    acquire_cube: AcquireCubeController
    recover: RecoverController
    score_game_piece: ScoreGamePieceController

    arm_controller: ArmController

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
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.1) * self.max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.1) * self.max_speed
        drive_z = -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * spin_rate
        local_driving = self.gamepad.getBButton()
        self.movement.set_input(vx=drive_x, vy=drive_y, vz=drive_z, local=local_driving)

        # Automation
        left_trigger = self.gamepad.getLeftTriggerAxis() > 0.3
        right_trigger = self.gamepad.getRightTriggerAxis() > 0.3
        if right_trigger:
            self.scoring.set_direction_to_find_place(True)
        if left_trigger:
            self.scoring.set_direction_to_find_place(False)
        self.scoring.autodrive = right_trigger or left_trigger

        # Intake
        if self.gamepad.getRightBumperPressed():
            self.scoring.wants_to_intake = True
        if self.gamepad.getLeftBumperPressed():
            self.scoring.wants_to_intake = False

        self.scoring.engage()

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
        self.gamepad.getRightTriggerAxis()
        self.gamepad.getLeftTriggerAxis()
        self.arm.unbrake()
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
            self.arm.go_to_setpoint(Setpoints.STOW)
        # left
        elif dpad_angle == 270:
            self.arm.go_to_setpoint(Setpoints.SCORE_CONE_MID)

        # Claw
        if self.gamepad.getYButton():
            self.gripper.set_solenoid = True
            self.gripper.close()
        if self.gamepad.getXButton():
            self.gripper.set_solenoid = True
            self.gripper.open()

        # LEDs
        if self.gamepad.getAButton():
            if dpad_angle == 0:
                self.status_lights.set_display_pattern(DisplayType.SOLID)
                self.status_lights.set_color([LedColors.OFF])
            if dpad_angle == 90 or dpad_angle == 270:
                self.status_lights.want_cube()
            if dpad_angle == 180:
                self.status_lights.cube_onboard()
        if self.gamepad.getBButton():
            if dpad_angle == 90:
                self.status_lights.want_cone_right()
            if dpad_angle == 270:
                self.status_lights.want_cone_left()
            if dpad_angle == 0:
                self.status_lights.set_display_pattern(DisplayType.SOLID)
                self.status_lights.set_color([LedColors.OFF])
            if dpad_angle == 180:
                self.status_lights.cone_onboard()

        if self.gamepad.getRightBumperPressed():
            self.intake.deploy()
        if self.gamepad.getLeftBumperPressed():
            self.intake.retract()

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
