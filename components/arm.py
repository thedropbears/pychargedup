from magicbot import feedback, tunable
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
from ids import CanIds, PcmChannels
import math
from wpilib import Solenoid, PneumaticsModuleType
from wpimath.controller import (
    ProfiledPIDController,
    ArmFeedforward,
    SimpleMotorFeedforwardMeters,
)
from wpimath.trajectory import TrapezoidProfile
from utilities.functions import clamp
from dataclasses import dataclass


class Arm:
    MIN_EXTENSION = 0.7  # meters
    MAX_EXTENSION = 1.3

    ROTATE_GEAR_RATIO = (60 / 25) * (60 / 25) * (70 / 20)
    SPOOL_DIAMETER = 0.05
    EXTEND_OUTPUT_RATIO = (1 / 7) * (math.pi * SPOOL_DIAMETER)  # converts to meters
    EXTEND_GRAVITY_FEEDFORWARD = 0

    # Angle soft limits
    MIN_ANGLE = math.radians(-230)
    MAX_ANGLE = math.radians(70)

    # how far either side of vertical will the arm retract if it is in
    # too avoid exceeding the max hieght
    UPRIGHT_ANGLE = math.radians(20)

    goal_angle = tunable(0.0)
    goal_extension = tunable(MIN_EXTENSION)
    # automatically retract the extension when rotating overhead
    do_auto_retract = tunable(False)

    def __init__(self) -> None:
        # Create rotation things
        # left motor is main
        self.rotation_motor = CANSparkMax(
            CanIds.Arm.rotation_left, CANSparkMax.MotorType.kBrushless
        )
        self.rotation_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.rotation_motor.setInverted(False)
        # setup second motor to follow first
        self._rotation_motor_right = CANSparkMax(
            CanIds.Arm.rotation_right, CANSparkMax.MotorType.kBrushless
        )
        self._rotation_motor_right.follow(self.rotation_motor, invert=True)
        self._rotation_motor_right.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self._rotation_motor_right.setInverted(False)

        self.rotation_encoder = self.rotation_motor.getAbsoluteEncoder(
            SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.rotation_encoder.setZeroOffset(0)
        self.rotation_encoder.setInverted(False)
        # output position = conversion factor * motor rotations
        self.rotation_encoder.setPositionConversionFactor(self.ROTATE_GEAR_RATIO)
        # also go from RPM to RPS
        self.rotation_encoder.setVelocityConversionFactor(self.ROTATE_GEAR_RATIO * 60)

        # TODO: get pid and feedforward values for arm and extension from sysid
        # running the controller on the rio rather than on the motor controller
        # to allow access to the velocity setpoint for feedforward
        rotation_constraints = TrapezoidProfile.Constraints(
            maxVelocity=3, maxAcceleration=3
        )
        self.rotation_controller = ProfiledPIDController(5, 0, 0, rotation_constraints)
        self.rotation_ff = ArmFeedforward(kS=0.1, kG=3, kV=2, kA=0)

        # Create extension things
        self.extension_motor = CANSparkMax(
            CanIds.Arm.extension, CANSparkMax.MotorType.kBrushless
        )
        self.extension_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.extension_motor.setInverted(False)
        self.extension_encoder = self.extension_motor.getEncoder()
        self.extension_encoder.setPositionConversionFactor(self.EXTEND_OUTPUT_RATIO)
        self.extension_encoder.setVelocityConversionFactor(
            self.EXTEND_OUTPUT_RATIO * 60
        )
        # assume retracted starting position
        self.extension_encoder.setPosition(self.MIN_EXTENSION)
        self.extension_controller = ProfiledPIDController(
            15, 0, 0, TrapezoidProfile.Constraints(maxVelocity=2, maxAcceleration=2)
        )
        self.extension_simple_ff = SimpleMotorFeedforwardMeters(0, 0.1, 0.1)

        self.brake_solenoid = Solenoid(
            PneumaticsModuleType.CTREPCM, PcmChannels.arm_brake
        )

    def execute(self) -> None:
        # Clamp extension to not break max height rules
        is_angle_forwards = math.copysign(self.get_angle() + math.pi / 2, 1)
        is_goal_forwards = math.copysign(self.goal_angle + math.pi / 2, 1)
        # if we plan on rotating overhead
        is_going_over = is_angle_forwards != is_goal_forwards
        # if we are currently overhead
        is_currently_up = (
            self.UPRIGHT_ANGLE > (self.get_angle() + math.pi / 2) > -self.UPRIGHT_ANGLE
        )
        should_retract = (is_going_over or is_currently_up) and self.do_auto_retract
        actual_extension_goal = (
            self.MIN_EXTENSION if should_retract else self.goal_extension
        )

        # Calculate extension motor output
        pid_output = self.extension_controller.calculate(
            self.get_extension(), actual_extension_goal
        )
        setpoint: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        self.calculate_extension_feedforward(setpoint.velocity)
        self.extension_motor.setVoltage(pid_output)

        if self.at_goal_angle() and self.is_angle_still():
            self.brake()
            self.rotation_motor.set(0)
            return

        # Calculate rotation motor output
        pid_output = self.rotation_controller.calculate(self.get_angle())
        setpoint = self.rotation_controller.getSetpoint()
        rotation_ff = self.calculate_rotation_feedforwards(setpoint.velocity)
        self.rotation_motor.set(pid_output + rotation_ff)

    def calculate_rotation_feedforwards(self, next_speed: float) -> float:
        """Calculate feedforwards voltage.
        next_speed: speed in rps"""
        accel = next_speed - self.get_arm_speed()
        return self.rotation_ff.calculate(self.get_angle(), self.get_arm_speed(), accel)

    def calculate_extension_feedforward(self, next_speed) -> float:
        extend_ff_G = self.EXTEND_GRAVITY_FEEDFORWARD * math.sin(-self.get_angle())
        extension_speed = self.extension_encoder.getVelocity()
        extend_ff_simple = self.extension_simple_ff.calculate(
            extension_speed, next_speed, 0.02
        )
        return extend_ff_G + extend_ff_simple

    @feedback
    def get_angle(self) -> float:
        """Get the position of the arm in in radians, 0 forwards, CCW down"""
        return self.rotation_encoder.getPosition()

    def get_arm_speed(self) -> float:
        """Get the speed of the arm in Rotations/s"""
        return self.rotation_encoder.getVelocity()

    @feedback
    def get_extension(self) -> float:
        """Gets the extension length in meters"""
        return self.MIN_EXTENSION + self.extension_encoder.getPosition()

    def set_angle(self, value: float) -> None:
        """Sets a goal angle to go to in radians, 0 forwards, CCW down"""
        self.goal_angle = clamp(value, self.MIN_ANGLE, self.MAX_ANGLE)

    def set_length(self, value: float) -> None:
        """Sets a goal length to go to in meters"""
        self.goal_extension = clamp(value, self.MIN_EXTENSION, self.MAX_EXTENSION)

    DEFAULT_ALLOWABLE_ANGLE_ERROR = math.radians(5)

    def at_goal_angle(
        self, allowable_error: float = DEFAULT_ALLOWABLE_ANGLE_ERROR
    ) -> bool:
        return abs(self.get_angle() - self.goal_angle) < allowable_error

    def at_goal_extension(self, allowable_error=0.05) -> bool:
        return abs(self.get_extension() - self.goal_extension) < allowable_error

    def is_angle_still(self, allowable_speed=0.01) -> bool:
        """Is the arm currently not moving, allowable speed is in Rotations/s"""
        return abs(self.get_arm_speed()) < allowable_speed

    def brake(self):
        self.brake_solenoid.set(True)

    def unbrake(self):
        self.brake_solenoid.set(False)

    def on_enable(self):
        self.extension_controller.reset(self.get_extension())
        self.rotation_controller.reset(self.get_angle())


@dataclass
class Setpoint:
    # arm angle in radians
    # angle of 0 points towards positive x i.e. at the intake
    # positive counterclockwise when looking at the left side of the robot
    angle: float
    # extension in meters, length from center of rotation to center of where pieces are held in the end effector
    extension: float

    def __init__(self, angle: float, extension: float):
        self.extension = clamp(extension, Arm.MIN_EXTENSION, Arm.MAX_EXTENSION)
        if angle > Arm.MAX_ANGLE:  # and (self.angle - math.tau) < Arm.MIN_ANGLE:
            angle -= math.tau
        self.angle = clamp(angle, Arm.MIN_ANGLE, Arm.MAX_ANGLE)

        if self.angle != angle or self.extension != extension:
            print(
                "SETPOINT WAS CLAMPED", (angle, extension), (self.angle, self.extension)
            )
            raise ValueError()

    @staticmethod
    def fromCartesian(x: float, z: float) -> "Setpoint":
        """Sets a cartesian goal reletive to the arms axle"""
        return Setpoint(math.atan2(-z, x), math.hypot(x, z))


class Setpoints:
    PICKUP_CONE = Setpoint(-math.pi, Arm.MIN_EXTENSION + 0.05)
    HANDOFF = Setpoint(Arm.MAX_ANGLE, Arm.MIN_EXTENSION)
    SCORE_CONE_MID = Setpoint.fromCartesian(-0.80, 0.12)
    SCORE_CUBE_MID = Setpoint.fromCartesian(-0.80, -0.20)
    SCORE_CONE_HIGH = Setpoint.fromCartesian(-1.22, 0.42)
    SCORE_CUBE_HIGH = Setpoint.fromCartesian(-1.22, 0.10)
