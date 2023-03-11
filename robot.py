#!/usr/bin/env python3

import wpilib
import wpilib.event
import magicbot
from wpimath.geometry import Quaternion, Rotation3d, Translation3d, Pose2d, Rotation2d

from controllers.movement import Movement
from controllers.arm import ArmController, Setpoints

from controllers.acquire_cone import AcquireConeController
from controllers.acquire_cube import AcquireCubeController
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController

from components.intake import Intake
from components.chassis import Chassis
from components.vision import VisualLocalizer
from components.arm import Arm
from components.gripper import Gripper
from components.leds import StatusLights, DisplayType, LedColors
from utilities.scalers import rescale_js
from utilities.game import is_red, get_node_location


class MyRobot(magicbot.MagicRobot):
    # Automations
    acquire_cone: AcquireConeController
    acquire_cube: AcquireCubeController
    score_game_piece: ScoreGamePieceController
    recover: RecoverController

    # Controllers
    movement: Movement
    arm: ArmController

    # Components
    chassis: Chassis
    arm_component: Arm
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
        # Right trigger events
        self.right_trigger_down_full = self.gamepad.rightTrigger(
            0.95, self.event_loop
        ).rising()
        self.right_trigger_down_half = self.gamepad.rightTrigger(0.05, self.event_loop)
        self.right_trigger_up = self.gamepad.rightTrigger(
            0.95, self.event_loop
        ).falling()

        # Left trigger events
        self.left_trigger_down_full = self.gamepad.leftTrigger(
            0.95, self.event_loop
        ).rising()
        self.left_trigger_down_half = self.gamepad.leftTrigger(
            0.05, self.event_loop
        ).rising()
        self.left_trigger_up = self.gamepad.leftTrigger(0.95, self.event_loop).falling()

        self.rumble_timer = wpilib.Timer()
        self.rumble_timer.start()
        self.rumble_duration = 0.0

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)
        self.target_node = self.field.getObject("target_node")

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
        self.last_dpad = -1

    def rumble_for(self, intensity: float, duration: float):
        self.rumble_duration = duration
        self.rumble_timer.reset()
        self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, intensity)

    def short_rumble(self):
        self.rumble_for(0.4, 0.1)

    def long_rumble(self):
        self.rumble_for(0.8, 0.3)

    def teleopInit(self) -> None:
        self.recover.engage()

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
        if (
            self.left_trigger_down_full.getAsBoolean()
            or self.right_trigger_down_full.getAsBoolean()
        ):
            self.acquire_cone.engage()
            self.long_rumble()
        if self.left_trigger_up.getAsBoolean() or self.right_trigger_up.getAsBoolean():
            self.acquire_cone.done()

        # Request cone / Set pickup side
        if self.left_trigger_down_half.getAsBoolean():
            self.acquire_cone.target_left()
            self.short_rumble()
        if self.right_trigger_down_half.getAsBoolean():
            self.acquire_cone.target_right()
            self.short_rumble()

        # Score, auto pick node
        if self.gamepad.getAButtonPressed() and not self.recover.is_executing:
            self.score_game_piece.score()

        # Intake
        if self.gamepad.getRightBumperPressed():
            self.acquire_cube.engage()
        if self.gamepad.getLeftBumperPressed():
            self.acquire_cube.done()
        # Intake manual grab
        if self.gamepad.getLeftStickButtonPressed():
            self.acquire_cube.override_cube_present = True
        # Intake clear, deploy intake and run backwards
        if self.gamepad.getRightStickButtonPressed():
            self.intake.run_backwards()
        # stop intake running backwards
        if self.gamepad.getRightStickButtonReleased():
            self.intake.deploy_without_running()

        # Request cube
        if self.gamepad.getXButtonPressed():
            self.status_lights.want_cube()

        # Run autobalance in teleop, for tuning gains in practice
        if self.gamepad.getYButtonPressed():
            self.arm.go_to_setpoint(Setpoints.PICKUP_CONE)

        # Stop controllers / Clear request
        if self.gamepad.getBButtonPressed():
            self.status_lights.off()
            self.arm_component.use_voltage = True
            self.cancel_controllers()

        dpad_angle = self.gamepad.getPOV()
        if dpad_angle != self.last_dpad:
            # Up, score closest high
            if dpad_angle == 0:
                self.score_game_piece.set_score_high()
            # Down, score closest mid
            elif dpad_angle == 180:
                self.score_game_piece.set_score_mid()
            # Right, manual arm to score high position
            if dpad_angle == 90:
                self.arm.go_to_setpoint(Setpoints.SCORE_CONE_HIGH)
            # Left, manual arm to score mid postion
            if dpad_angle == 270:
                self.arm.go_to_setpoint(Setpoints.SCORE_CONE_MID)
        self.last_dpad = dpad_angle

        # Manual overrides
        # Claw
        if self.gamepad.getStartButtonPressed():
            self.gripper.close()
        if self.gamepad.getBackButtonPressed():
            self.gripper.open()

        # stop rumble after time
        if self.rumble_timer.hasElapsed(self.rumble_duration):
            self.gamepad.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0)

        # Show node to be scored at on dashbaord
        n = get_node_location(
            self.score_game_piece._get_closest(self.score_game_piece.prefered_row)
        )
        self.target_node.setPose(Pose2d(n.toTranslation2d(), Rotation2d()))

    def testInit(self) -> None:
        self.arm_component.on_enable()
        self.port_localizer.add_to_estimator = True
        self.starboard_localizer.add_to_estimator = True

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
            # up
            if dpad_angle == 0:
                self.arm.go_to_setpoint(Setpoints.SCORE_CONE_HIGH)
            # down
            elif dpad_angle == 180:
                self.arm.go_to_setpoint(Setpoints.SCORE_CONE_MID)
            # right
            elif dpad_angle == 90:
                self.arm.go_to_setpoint(Setpoints.PREPARE_PICKUP_CONE)
            # left
            elif dpad_angle == 270:
                self.arm.go_to_setpoint(Setpoints.STOW)

        # State machines
        if self.gamepad.getXButton():
            if self.gamepad.getLeftBumperPressed():
                self.acquire_cone.target_left()
            if self.gamepad.getRightBumperPressed():
                self.acquire_cone.target_right()
            if dpad_angle == 180:
                self.acquire_cone.engage("deploying_arm")
            if dpad_angle == 0:
                self.acquire_cube.engage()
            if dpad_angle == 90 or dpad_angle == 270:
                # self.recover.engage()
                # Better to call cancel() so that all other SMs
                # are cancelled before recover runs
                self.cancel_controllers()
            if self.gamepad.getLeftStickButtonPressed():
                self.score_game_piece.engage()

        # Cancel any running controllers
        if self.gamepad.getBackButtonPressed():
            self.cancel_controllers()

        # Tick the controllers
        # These will only do anything if engage() has been called on them
        self.arm.execute()
        self.acquire_cone.execute()
        self.acquire_cube.execute()
        self.score_game_piece.execute()
        self.recover.execute()

        self.arm_component.execute()
        self.intake.execute()
        self.gripper.execute()
        self.chassis.update_odometry()
        self.port_localizer.execute()
        self.starboard_localizer.execute()
        self.status_lights.execute()

    def cancel_controllers(self):
        self.acquire_cone.done()
        self.acquire_cube.done()
        self.score_game_piece.done()
        self.status_lights.off()
        self.recover.engage()
        self.movement.done()

    def disabledInit(self) -> None:
        self.status_lights.set_display_pattern(DisplayType.PULSE)

    def disabledPeriodic(self) -> None:
        self.chassis.update_odometry()
        # This could be set in init, but it is more responsive if we do it here
        if is_red():
            self.status_lights.set_color(LedColors.DIM_RED)
        else:
            self.status_lights.set_color(LedColors.DIM_BLUE)

        self.status_lights.execute()
        self.port_localizer.execute()
        self.starboard_localizer.execute()
        self.arm_component.update_display()

    def autonomousInit(self) -> None:
        self.port_localizer.add_to_estimator = True
        self.starboard_localizer.add_to_estimator = True


if __name__ == "__main__":
    wpilib.run(MyRobot)
