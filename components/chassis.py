from logging import Logger
import math
import time
from typing import Optional

import ctre.sensors
import magicbot
import navx
import wpilib
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.interpolation import TimeInterpolatablePose2dBuffer
from wpimath.controller import SimpleMotorFeedforwardMeters

from utilities.functions import constrain_angle, rate_limit_module
from utilities.ctre import FALCON_CPR, FALCON_FREE_RPS
from ids import CancoderIds, TalonIds


class SwerveModule:
    DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    STEER_GEAR_RATIO = (14 / 50) * (10 / 60)
    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    DRIVE_MOTOR_REV_TO_METRES = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO
    METRES_TO_DRIVE_COUNTS = FALCON_CPR / DRIVE_MOTOR_REV_TO_METRES
    DRIVE_COUNTS_TO_METRES = DRIVE_MOTOR_REV_TO_METRES / FALCON_CPR

    STEER_MOTOR_REV_TO_RAD = math.tau * STEER_GEAR_RATIO
    STEER_COUNTS_TO_RAD = STEER_MOTOR_REV_TO_RAD / FALCON_CPR
    STEER_RAD_TO_COUNTS = FALCON_CPR / STEER_MOTOR_REV_TO_RAD

    # limit the acceleration of the commanded speeds of the robot to what is actually
    # achiveable without the wheels slipping. This is done to improve odometry
    # TODO: measure this empirically
    accel_limit = 5  # m/s^2

    def __init__(
        self,
        x: float,
        y: float,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
        steer_reversed=True,
        drive_reversed=False,
    ):
        """
        x, y: where the module is relative to the center of the robot
        *_id: can ids of steer and drive motors and absolute encoder
        """
        self.translation = Translation2d(x, y)
        self.state = SwerveModuleState(0, Rotation2d(0))
        self.do_smooth = True

        # Create Motor and encoder objects
        self.steer = ctre.WPI_TalonFX(steer_id)
        self.drive = ctre.WPI_TalonFX(drive_id)
        self.drive_id = drive_id
        self.encoder = ctre.sensors.CANCoder(encoder_id)

        # Reduce CAN status frame rates before configuring
        self.steer.setStatusFramePeriod(
            ctre.StatusFrameEnhanced.Status_1_General, 250, 10
        )
        self.drive.setStatusFramePeriod(
            ctre.StatusFrameEnhanced.Status_1_General, 250, 10
        )

        # Configure steer motor
        self.steer.setNeutralMode(ctre.NeutralMode.Brake)
        self.steer.setInverted(steer_reversed)
        self.steer.config_kP(0, 0.15035, 10)
        self.steer.config_kI(0, 0, 10)
        self.steer.config_kD(0, 5.6805, 10)
        self.steer.configAllowableClosedloopError(
            0, self.STEER_RAD_TO_COUNTS * math.radians(4)
        )
        self.steer.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )

        # Configure drive motor
        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive.configVoltageCompSaturation(11.5, timeoutMs=10)
        self.drive.enableVoltageCompensation(True)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.18877, kV=2.7713, kA=0.18824)
        self.drive.configVelocityMeasurementPeriod(
            ctre.SensorVelocityMeasPeriod.Period_5Ms
        )
        self.drive.configVelocityMeasurementWindow(8)
        self.drive.config_kP(0, 0.011489, 10)
        self.drive.config_kI(0, 0, 10)
        self.drive.config_kD(0, 0, 10)

    def get_angle_absolute(self) -> float:
        """Gets steer angle (radians) from absolute encoder"""
        return math.radians(self.encoder.getAbsolutePosition())

    def get_angle_integrated(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.getSelectedSensorPosition() * self.STEER_COUNTS_TO_RAD

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_integrated())

    def get_speed(self) -> float:
        # velocity is in counts / 100ms, return in m/s
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_COUNTS_TO_METRES * 10

    def get_distance_traveled(self) -> float:
        return self.drive.getSelectedSensorPosition() * self.DRIVE_COUNTS_TO_METRES

    def set(self, desired_state: SwerveModuleState):
        # smooth wheel velocity vector
        if self.do_smooth:
            self.state = rate_limit_module(self.state, desired_state, self.accel_limit)
        else:
            self.state = desired_state
        self.state = SwerveModuleState.optimize(self.state, self.get_rotation())

        if abs(self.state.speed) < 0.01:
            self.drive.set(ctre.ControlMode.Velocity, 0)
            self.steer.set(ctre.ControlMode.PercentOutput, 0)
            return

        current_angle = self.get_angle_integrated()
        target_displacement = constrain_angle(
            self.state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer.set(
            ctre.ControlMode.Position, target_angle * self.STEER_RAD_TO_COUNTS
        )

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = self.state.speed * math.cos(target_displacement) ** 2
        speed_volt = self.drive_ff.calculate(target_speed)
        self.drive.set(
            ctre.ControlMode.Velocity,
            target_speed * self.METRES_TO_DRIVE_COUNTS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / 12,
        )

    def sync_steer_encoders(self) -> None:
        self.steer.setSelectedSensorPosition(
            self.get_angle_absolute() * self.STEER_RAD_TO_COUNTS
        )

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance_traveled(), self.get_rotation())

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class Chassis:
    # metres between centre of left and right wheels
    TRACK_WIDTH = 0.54665
    # metres between centre of front and back wheels
    WHEEL_BASE = 0.68665

    # size including bumpers
    LENGTH = 1.0105
    WIDTH = 0.8705

    # maxiumum speed for any wheel
    max_wheel_speed = FALCON_FREE_RPS * SwerveModule.DRIVE_MOTOR_REV_TO_METRES

    control_loop_wait_time: float

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    send_modules = magicbot.tunable(False)
    do_fudge = magicbot.tunable(True)
    do_smooth = magicbot.tunable(True)

    def __init__(self) -> None:
        self.pose_history = TimeInterpolatablePose2dBuffer(2)
        self.last_pose = Pose2d()
        self.translation_velocity = Translation2d()
        self.rotation_velocity = Rotation2d()
        self.last_time = time.monotonic()

    def setup(self) -> None:
        self.imu = navx.AHRS.create_spi()

        self.modules = [
            # Front Left
            SwerveModule(
                self.WHEEL_BASE / 2,
                self.TRACK_WIDTH / 2,
                TalonIds.drive_1,
                TalonIds.steer_1,
                CancoderIds.swerve_1,
            ),
            # Back Left
            SwerveModule(
                -self.WHEEL_BASE / 2,
                self.TRACK_WIDTH / 2,
                TalonIds.drive_2,
                TalonIds.steer_2,
                CancoderIds.swerve_2,
            ),
            # Back Right
            SwerveModule(
                -self.WHEEL_BASE / 2,
                -self.TRACK_WIDTH / 2,
                TalonIds.drive_3,
                TalonIds.steer_3,
                CancoderIds.swerve_3,
            ),
            # Front Right
            SwerveModule(
                self.WHEEL_BASE / 2,
                -self.TRACK_WIDTH / 2,
                TalonIds.drive_4,
                TalonIds.steer_4,
                CancoderIds.swerve_4,
            ),
        ]

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )
        self.sync_all()
        self.imu.zeroYaw()
        self.imu.resetDisplacement()
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.imu.getRotation2d(),
            self.get_module_positions(),
            Pose2d(3, 0, 0),
            stateStdDevs=(0.05, 0.05, 0.01),
            visionMeasurementStdDevs=(0.4, 0.4, math.inf),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.module_objs: list[wpilib.FieldObject2d] = []
        for idx, _module in enumerate(self.modules):
            self.module_objs.append(self.field.getObject("s_module_" + str(idx)))
        self.set_pose(Pose2d(4, 3.5, Rotation2d.fromDegrees(180)))

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    def execute(self) -> None:
        # rotate desired velocity to compensate for skew caused by discretization
        # see https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892/
        if self.do_fudge:
            # in the sim i found using 5 instead of 0.5 did a lot better
            desired_speed_translation = Translation2d(
                self.chassis_speeds.vx, self.chassis_speeds.vy
            ).rotateBy(
                Rotation2d(-self.chassis_speeds.omega * 5 * self.control_loop_wait_time)
            )
            desired_speeds = ChassisSpeeds(
                desired_speed_translation.x,
                desired_speed_translation.y,
                self.chassis_speeds.omega,
            )
        else:
            desired_speeds = self.chassis_speeds

        desired_states = self.kinematics.toSwerveModuleStates(desired_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed
        )
        for state, module in zip(desired_states, self.modules):
            module.do_smooth = self.do_smooth
            module.set(state)

        self.update_odometry()
        self.update_pose_history()
        self.last_time = time.monotonic()

    @magicbot.feedback
    def get_imu_speed(self) -> float:
        return math.hypot(self.imu.getVelocityX(), self.imu.getVelocityY())

    def get_velocity(self) -> ChassisSpeeds:
        self.local_speed = self.kinematics.toChassisSpeeds(
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self.local_speed, -self.get_rotation()
        )

    def update_odometry(self) -> None:
        self.estimator.update(self.imu.getRotation2d(), self.get_module_positions())
        self.field_obj.setPose(self.get_pose())
        if self.send_modules:
            robot_location = self.estimator.getEstimatedPosition()
            for idx, module in enumerate(self.modules):
                module_location = (
                    robot_location.translation()
                    + module.translation.rotateBy(robot_location.rotation())
                )
                module_rotation = module.get_rotation().rotateBy(
                    robot_location.rotation()
                )
                self.module_objs[idx].setPose(Pose2d(module_location, module_rotation))

    def update_pose_history(self) -> None:
        pose = self.get_pose()
        self.pose_history.addSample(wpilib.Timer.getFPGATimestamp(), pose)
        self.last_pose = pose

    def sync_all(self) -> None:
        for m in self.modules:
            m.sync_steer_encoders()

    def set_pose(self, pose: Pose2d) -> None:
        self.pose_history.clear()
        self.estimator.resetPosition(
            self.imu.getRotation2d(), self.get_module_positions(), pose
        )
        self.update_pose_history()
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def zero_yaw(self) -> None:
        """Sets pose to current pose but with a heading of zero"""
        cur_pose = self.estimator.getEstimatedPosition()
        self.estimator.resetPosition(
            self.imu.getRotation2d(),
            self.get_module_positions(),
            Pose2d(cur_pose.translation(), Rotation2d(0)),
        )

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.modules[0].get_position(),
            self.modules[1].get_position(),
            self.modules[2].get_position(),
            self.modules[3].get_position(),
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to ???"""
        return self.estimator.getEstimatedPosition()

    def get_gyro_pose(self) -> Pose2d:
        return Pose2d(
            self.imu.getDisplacementX(),
            self.imu.getDisplacementY(),
            self.imu.getRotation2d(),
        )

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    def get_pose_at(self, t: float) -> Pose2d:
        """Gets where the robot was at t seconds in the past"""
        pose = self.pose_history.sample(t)
        # sample returns None if there is nothing in the history
        if pose is None:
            return self.get_pose()
        return pose

    def robot_to_world(
        self, offset: Translation2d, robot: Optional[Pose2d] = None
    ) -> Pose2d:
        """Transforms a translation from robot space to world space (e.g. turret position)"""
        if robot is None:
            robot = self.estimator.getEstimatedPosition()
        return Pose2d(
            robot.translation() + offset.rotateBy(robot.rotation()), robot.rotation()
        )

    def on_enable(self) -> None:
        # update the odometry so the pose estimator dosent have an empty buffer
        self.update_odometry()
