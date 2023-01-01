from logging import Logger
import math
import time
from typing import Optional

import ctre
import magicbot
import navx
import wpilib
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.interpolation import TimeInterpolatablePose2dBuffer
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.filter import SlewRateLimiter

from utilities.functions import constrain_angle, rate_limit_2d
from utilities.ctre import FALCON_CPR, FALCON_FREE_RPS
from ids import CanIds


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

        # Create Motor and encoder objects
        self.steer = ctre.WPI_TalonFX(steer_id)
        self.drive = ctre.WPI_TalonFX(drive_id)
        self.encoder = ctre.CANCoder(encoder_id)

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
            0, self.STEER_RAD_TO_COUNTS * math.radians(3)
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

        # Configure encoder
        self.encoder.configFeedbackCoefficient(
            math.tau / 4096, "rad", ctre.SensorTimeBase.PerSecond, timeoutMs=10
        )

    def get_angle_absolute(self) -> float:
        """Gets steer angle from absolute encoder"""
        return self.encoder.getAbsolutePosition()

    def get_angle_integrated(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.getSelectedSensorPosition() * self.STEER_COUNTS_TO_RAD

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_integrated())

    def get_speed(self) -> float:
        # velocity is in counts / 100ms, return in m/s
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_COUNTS_TO_METRES * 10

    def set(self, desired_state: SwerveModuleState):
        if abs(desired_state.speed) < 1e-3:
            self.drive.set(ctre.ControlMode.Velocity, 0)
            self.steer.set(ctre.ControlMode.Velocity, 0)
            return

        current_angle = self.get_angle_integrated()
        target_displacement = constrain_angle(
            desired_state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer.set(
            ctre.ControlMode.Position, target_angle * self.STEER_RAD_TO_COUNTS
        )

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = desired_state.speed * math.cos(target_displacement) ** 2
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

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class Chassis:
    # assumes square chassis
    WIDTH = 0.6167  # meters between modules from CAD
    WHEEL_DIST = math.sqrt(2) * WIDTH / 2
    # maxiumum speed for any wheel
    max_wheel_speed = FALCON_FREE_RPS * SwerveModule.DRIVE_MOTOR_REV_TO_METRES
    # limit the acceleration of the robot to what is actually achiveable
    # without the drive wheels slipping to improve odometry
    # TODO: measure this empirically
    accel_limit = 15

    control_loop_wait_time: float

    # the speeds we want to be at, will be smoothed to the speeds that are commanded
    desired_chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    def __init__(self) -> None:
        self.pose_history = TimeInterpolatablePose2dBuffer(2)
        self.last_pose = Pose2d()
        self.translation_velocity = Translation2d()
        self.rotation_velocity = Rotation2d()
        self.last_time = time.monotonic()
        # actual speeds that are commanded
        self.commanded_chassis_speeds = ChassisSpeeds(0, 0, 0)

    def setup(self) -> None:
        self.imu = navx.AHRS.create_spi()

        self.modules = [
            # Front Left
            SwerveModule(
                self.WIDTH / 2,
                self.WIDTH / 2,
                CanIds.Chassis.drive_1,
                CanIds.Chassis.steer_1,
                CanIds.Chassis.encoder_1,
            ),
            # Back Left
            SwerveModule(
                -self.WIDTH / 2,
                self.WIDTH / 2,
                CanIds.Chassis.drive_2,
                CanIds.Chassis.steer_2,
                CanIds.Chassis.encoder_2,
            ),
            # Back Right
            SwerveModule(
                -self.WIDTH / 2,
                -self.WIDTH / 2,
                CanIds.Chassis.drive_3,
                CanIds.Chassis.steer_3,
                CanIds.Chassis.encoder_3,
                drive_reversed=True,
            ),
            # Front Right
            SwerveModule(
                self.WIDTH / 2,
                -self.WIDTH / 2,
                CanIds.Chassis.drive_4,
                CanIds.Chassis.steer_4,
                CanIds.Chassis.encoder_4,
                drive_reversed=True,
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
        self.estimator = SwerveDrive4PoseEstimator(
            self.imu.getRotation2d(),
            Pose2d(0, 0, 0),
            self.kinematics,
            stateStdDevs=(0.1, 0.1, math.radians(5)),
            localMeasurementStdDevs=(0.01,),
            visionMeasurementStdDevs=(0.5, 0.5, 0.2),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.set_pose(Pose2d(2, 0, Rotation2d.fromDegrees(180)))

    def drive_field(self, vx: float, vy: float, omega: float, smooth=True) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current_heading)
        self._drive(speeds, smooth)

    def drive_local(self, vx: float, vy: float, omega: float, smooth=True) -> None:
        """Robot oriented drive commands"""
        speeds = ChassisSpeeds(vx, vy, omega)
        self._drive(speeds, smooth)

    def _drive(self, speeds: ChassisSpeeds, smooth: bool) -> None:
        if not smooth:
            self.commanded_chassis_speeds = speeds
        self.desired_chassis_speeds = speeds

    def execute(self) -> None:
        # since the point of this smoothing is to improve odometry it dosent limit rotation
        # beacuse the gyro is unaffected by wheel slip
        self.commanded_chassis_speeds = rate_limit_2d(
            self.commanded_chassis_speeds, self.desired_chassis_speeds, self.accel_limit
        )
        desired_states = self.kinematics.toSwerveModuleStates(self.commanded_chassis_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed
        )
        for state, module in zip(desired_states, self.modules):
            state = SwerveModuleState.optimize(state, module.get_rotation())
            module.set(state)

        self.update_odometry()

        # add to prevent division by 0
        dt = min(time.monotonic() - self.last_time, 0.1) + 1e-10
        # rotation2d and translation2d have mul but not div
        control_rate = 1 / dt
        chassis_speeds = self.kinematics.toChassisSpeeds(
            (
                self.modules[0].get(),
                self.modules[1].get(),
                self.modules[2].get(),
                self.modules[3].get(),
            )
        )

        self.update_pose_history()
        self.last_time = time.monotonic()

    def update_odometry(self) -> None:
        self.estimator.update(
            self.imu.getRotation2d(),
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )
        self.field_obj.setPose(self.get_pose())

    def update_pose_history(self) -> None:
        pose = self.get_pose()
        self.pose_history.addSample(wpilib.Timer.getFPGATimestamp(), pose)
        self.last_pose = pose

    def sync_all(self) -> None:
        for m in self.modules:
            m.sync_steer_encoders()

    def set_pose(self, pose: Pose2d) -> None:
        self.pose_history.clear()
        self.estimator.resetPosition(pose, self.imu.getRotation2d())
        self.update_pose_history()
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def zero_yaw(self) -> None:
        """Sets pose to current pose but with a heading of zero"""
        cur_pose = self.estimator.getEstimatedPosition()
        self.estimator.resetPosition(
            Pose2d(cur_pose.translation(), Rotation2d(0)), self.imu.getRotation2d()
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to the goal."""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    def get_pose_at(self, t: float) -> Pose2d:
        """Gets where the robot was at t"""
        return self.pose_history.sample(t)

    def robot_to_world(
        self, offset: Translation2d, robot: Optional[Pose2d] = None
    ) -> Pose2d:
        """Transforms a translation from robot space to world space (e.g. turret position)"""
        if robot is None:
            robot = self.estimator.getEstimatedPosition()
        return Pose2d(
            robot.translation() + offset.rotateBy(robot.rotation()), robot.rotation()
        )
