from magicbot import feedback, tunable
import wpilib
from ids import SparkMaxIds, PhChannels, DioChannels
from wpilib import (
    Solenoid,
    PneumaticsModuleType,
    DutyCycleEncoder,
)
from wpimath.controller import (
    ProfiledPIDController,
    ArmFeedforward,
    SimpleMotorFeedforwardMeters,
)
from wpimath.trajectory import TrapezoidProfile
from utilities.functions import clamp
import rev
import math


MIN_EXTENSION = 0.9  # meters
MAX_EXTENSION = 1.25

# Angle soft limits
MIN_ANGLE = math.radians(-180)
MAX_ANGLE = math.radians(60)


class Arm:
    # height of the center of rotation off the ground
    PIVOT_HEIGHT = 0.990924
    PIVOT_X = -0.283562

    ANGLE_ERROR_TOLERANCE = math.radians(2)
    ANGLE_BRAKING_ERROR_TOLERANCE = math.radians(4)
    EXTENSION_ERROR_TOLERANCE = 0.01
    EXTENSION_BRAKING_ERROR_TOLERANCE = 0.02

    STILL_ROTATION_SPEED_TOLERANCE = 0.1
    STILL_EXTENSION_SPEED_TOLERANCE = 0.05

    ROTATE_GEAR_RATIO = (74 / 14) * (82 / 26) * (42 / 18)
    SPOOL_CIRCUMFERENCE = 42 * 0.005  # 42t x 5mm
    EXTEND_GEAR_RATIO = (7 / 1) * (4 / 1) * (34 / 18)
    # converts from motor rotations to meters
    EXTEND_OUTPUT_RATIO = SPOOL_CIRCUMFERENCE / EXTEND_GEAR_RATIO
    EXTEND_GRAVITY_FEEDFORWARD = 0
    ROTATE_GRAVITY_FEEDFORWARDS = 2.5

    ARM_ENCODER_ANGLE_OFFSET = 0.325  # rotations 0-1

    goal_angle = tunable(0.0)
    goal_extension = tunable(MIN_EXTENSION)

    control_loop_wait_time: float

    def __init__(self) -> None:
        # Create rotation things
        self.rotation_motor = rev.CANSparkMax(
            SparkMaxIds.arm_rotation_main, rev.CANSparkMax.MotorType.kBrushless
        )
        self.rotation_motor.restoreFactoryDefaults()
        self.rotation_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.rotation_motor.setInverted(True)
        self.relative_encoder = self.rotation_motor.getEncoder()
        self.relative_encoder.setPositionConversionFactor(self.ROTATE_GEAR_RATIO)
        self.relative_encoder.setVelocityConversionFactor(
            1 / self.ROTATE_GEAR_RATIO / 60
        )

        self.absolute_encoder = DutyCycleEncoder(DioChannels.arm_absolute_encoder)
        self.absolute_encoder.setDistancePerRotation(math.tau)
        self.absolute_encoder.setPositionOffset(self.ARM_ENCODER_ANGLE_OFFSET)
        self.runtime_offset = 0.0

        # TODO: get pid and feedforward values for arm and extension from sysid
        # running the controller on the rio rather than on the motor controller
        # to allow access to the velocity setpoint for feedforward
        rotation_constraints = TrapezoidProfile.Constraints(
            maxVelocity=3, maxAcceleration=4
        )
        self.rotation_controller = ProfiledPIDController(33, 0, 1, rotation_constraints)
        wpilib.SmartDashboard.putData(self.rotation_controller)
        self.rotation_ff = ArmFeedforward(kS=0, kG=0, kV=1, kA=0.1)
        self.rotation_last_setpoint_vel = 0

        # Create extension things
        self.extension_motor = rev.CANSparkMax(
            SparkMaxIds.arm_extension, rev.CANSparkMax.MotorType.kBrushless
        )
        self.extension_motor.restoreFactoryDefaults()
        self.extension_motor.setSmartCurrentLimit(20)
        self.extension_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.extension_motor.setInverted(True)
        self.extension_encoder = self.extension_motor.getEncoder()
        self.extension_encoder.setPositionConversionFactor(self.EXTEND_OUTPUT_RATIO)
        self.extension_encoder.setVelocityConversionFactor(
            self.EXTEND_OUTPUT_RATIO / 60
        )
        # assume retracted starting position
        self.extension_encoder.setPosition(MIN_EXTENSION)
        self.extension_controller = ProfiledPIDController(
            50, 0, 1, TrapezoidProfile.Constraints(maxVelocity=1.0, maxAcceleration=4.0)
        )
        wpilib.SmartDashboard.putData(self.rotation_controller)
        self.extension_simple_ff = SimpleMotorFeedforwardMeters(kS=0, kV=2, kA=0.2)
        self.extension_last_setpoint_vel = 0

        self.rotation_brake_solenoid = Solenoid(
            PneumaticsModuleType.REVPH, PhChannels.arm_brake
        )
        self.extension_brake_solenoid = Solenoid(
            PneumaticsModuleType.REVPH, PhChannels.arm_extension_brake
        )

        # Create arm display
        self.arm_mech2d = wpilib.Mechanism2d(5, 3)
        arm_pivot = self.arm_mech2d.getRoot("ArmPivot", 2.5, self.PIVOT_HEIGHT)
        arm_pivot.appendLigament("ArmTower", self.PIVOT_HEIGHT - 0.05, -90)
        self.arm_ligament = arm_pivot.appendLigament("Arm", MIN_EXTENSION, 0)
        self.arm_goal_ligament = arm_pivot.appendLigament(
            "Arm_goal",
            MIN_EXTENSION,
            0,
            3,
            wpilib.Color8Bit(117, 68, 26),
        )
        self.arm_extend_ligament = self.arm_ligament.appendLigament(
            "ArmExtend",
            0.5,
            0,
            8,
            wpilib.Color8Bit(137, 235, 52),
        )
        self.arm_extend_goal_ligament = self.arm_ligament.appendLigament(
            "ArmExtend_goal",
            0.1,
            0,
            4,
            wpilib.Color8Bit(68, 117, 26),
        )

        # Hall effector
        self.extension_wall_switch_forward = wpilib.DigitalInput(
            DioChannels.arm_wall_pickup_switch
        )
        self.extension_limit_switch_reverse = (
            self.extension_motor.getReverseLimitSwitch(
                rev.SparkMaxLimitSwitch.Type.kNormallyOpen
            )
        )
        self.extension_limit_switch_reverse.enableLimitSwitch(True)

        self.use_voltage = False
        self.voltage = 0.0

        wpilib.SmartDashboard.putData("Arm sim", self.arm_mech2d)

    def setup(self) -> None:
        self.set_length(self.get_extension())

    def update_display(self) -> None:
        extend_state: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        rotate_state = self.rotation_controller.getSetpoint()

        # Update display
        self.arm_ligament.setAngle(-math.degrees(self.get_angle()))
        self.arm_goal_ligament.setAngle(-math.degrees(rotate_state.position))
        self.arm_extend_ligament.setLength(self.get_extension() - MIN_EXTENSION)
        self.arm_extend_goal_ligament.setLength(extend_state.position - MIN_EXTENSION)

    def execute(self) -> None:
        self.update_display()

        if self.use_voltage:
            self.extension_motor.setVoltage(self.voltage)
            self.unbrake_extension()
            self.rotation_motor.setVoltage(0)
            self.brake_rotation()
            return

        pid_output = self.extension_controller.calculate(
            self.get_extension(), self.goal_extension
        )
        # Extension
        if self.at_goal_extension() and self.is_extension_still():
            self.brake_extension()
            self.extension_motor.setVoltage(0)
        else:
            self.unbrake_extension()
            self.extension_motor.setVoltage(pid_output)

        # Calculate rotation motor output
        pid_output = self.rotation_controller.calculate(
            self.get_angle(), self.goal_angle
        )
        ff_output = self.calculate_rotation_feedforwards()
        # Rotation
        if self.at_goal_angle() and self.is_angle_still():
            self.brake_rotation()
            self.rotation_motor.setVoltage(0)
        else:
            self.unbrake_rotation()
            self.rotation_motor.setVoltage(pid_output + ff_output)

    def calculate_rotation_feedforwards(self) -> float:
        """Calculate feedforwards voltage.
        next_speed: speed in rps"""
        state: TrapezoidProfile.State = self.rotation_controller.getSetpoint()
        accel = (
            state.velocity - self.rotation_last_setpoint_vel
        ) / self.control_loop_wait_time
        self.rotation_last_setpoint_vel = state.velocity
        return self.rotation_ff.calculate(self.get_angle(), state.velocity, accel)

    def calculate_extension_feedforward(self) -> float:
        state: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        accel = (
            state.velocity - self.extension_last_setpoint_vel
        ) / self.control_loop_wait_time
        self.extension_last_setpoint_vel = state.velocity
        extend_ff_simple = self.extension_simple_ff.calculate(state.velocity, accel)
        extend_ff_G = self.EXTEND_GRAVITY_FEEDFORWARD * math.sin(self.get_angle())
        return extend_ff_G + extend_ff_simple

    @feedback
    def get_angle(self) -> float:
        """Get the position of the arm in in radians, 0 forwards, CCW down"""
        return self.absolute_encoder.getDistance() + self.runtime_offset

    @feedback
    def get_angle_deg(self) -> float:
        return math.degrees(self.get_angle())

    @feedback
    def get_raw_angle(self) -> float:
        return self.absolute_encoder.get()

    def get_arm_speed(self) -> float:
        """Get the speed of the arm in Rotations/s"""
        # uses the relative encoder beacuse the absolute one dosent report velocity
        return self.relative_encoder.getVelocity()

    @feedback
    def get_extension(self) -> float:
        """Gets the extension length in meters from axle"""
        return self.extension_encoder.getPosition()

    def get_extension_speed(self) -> float:
        """Gets the extension speed in m/s"""
        return self.extension_encoder.getVelocity()

    def is_extension_still(self) -> bool:
        return abs(self.get_extension_speed()) < 0.1

    @feedback
    def get_wall_switch(self) -> bool:
        return not self.extension_wall_switch_forward.get()

    def set_at_min_extension(self) -> None:
        """Sets the extension position to the min extension"""
        self.extension_encoder.setPosition(MIN_EXTENSION)

    @feedback
    def is_retracted(self) -> bool:
        return self.extension_limit_switch_reverse.get()

    def get_voltage(self) -> float:
        """Get the current voltage for the extension motor"""
        return self.voltage

    def get_use_voltage(self) -> bool:
        """Should use the `self.voltage` value for the extension motor"""
        return self.use_voltage

    def set_angle(self, value: float) -> None:
        """Sets a goal angle to go to in radians, 0 forwards, CCW down"""
        self.goal_angle = clamp(value, MIN_ANGLE, MAX_ANGLE)

    def set_voltage(self, value: float) -> None:
        """Sets a voltage for the extension motor, will only activate if use_voltage is True"""
        self.voltage = value

    def set_use_voltage(self, value: bool) -> None:
        """Should override calculations for voltage"""
        if self.use_voltage and not value:
            self.extension_controller.reset(self.get_extension())
        self.use_voltage = value

    def set_length(self, value: float) -> None:
        """Sets a goal length to go to in meters"""
        self.goal_extension = clamp(value, MIN_EXTENSION, MAX_EXTENSION)

    @feedback
    def at_goal_angle(self) -> bool:
        return self.at_angle(self.goal_angle)

    def at_angle(self, angle: float) -> bool:
        tolerance = (
            self.ANGLE_BRAKING_ERROR_TOLERANCE
            if self.is_braking_rotation()
            else self.ANGLE_ERROR_TOLERANCE
        )
        error = abs(self.get_angle() - angle)
        return error < tolerance

    @feedback
    def at_goal_extension(self) -> bool:
        return self.at_extension(self.goal_extension)

    def at_extension(self, extension: float) -> bool:
        tolerance = (
            self.EXTENSION_BRAKING_ERROR_TOLERANCE
            if self.is_braking_extension()
            else self.EXTENSION_ERROR_TOLERANCE
        )
        error = abs(self.get_extension() - extension)
        movement_done = self.extension_controller.getSetpoint().position == extension
        return error < tolerance and movement_done

    def is_angle_still(self) -> bool:
        """Is the arm currently not moving, allowable speed is in Rotations/s"""
        return abs(self.get_arm_speed()) < self.STILL_ROTATION_SPEED_TOLERANCE

    def at_goal(self) -> bool:
        return self.at_pose(self.goal_angle, self.goal_extension)

    def at_pose(self, angle: float, extension: float) -> bool:
        return (
            self.at_extension(extension)
            and self.at_angle(angle)
            and self.is_angle_still()
        )

    def brake_rotation(self) -> None:
        self.rotation_brake_solenoid.set(False)

    def unbrake_rotation(self) -> None:
        self.rotation_brake_solenoid.set(True)

    def brake_extension(self) -> None:
        self.extension_brake_solenoid.set(False)

    def unbrake_extension(self) -> None:
        self.extension_brake_solenoid.set(True)

    def is_braking_rotation(self) -> bool:
        return not self.rotation_brake_solenoid.get()

    def is_braking_extension(self) -> bool:
        return not self.extension_brake_solenoid.get()

    def on_enable(self) -> None:
        if self.get_angle() > math.pi / 2:
            self.runtime_offset = -math.tau
        self.reset_controllers()
        self.set_length(self.get_extension())
        self.set_angle(self.get_angle())

    def stop(self) -> None:
        self.rotation_motor.set(0)
        self.extension_motor.set(0)

    def reset_controllers(self) -> None:
        self.extension_controller.reset(self.get_extension())
        self.rotation_controller.reset(self.get_angle())

    @feedback
    def angle_error(self) -> float:
        return self.rotation_controller.getPositionError()

    @feedback
    def get_rotation_output(self) -> float:
        return self.rotation_motor.getAppliedOutput()

    @feedback
    def get_extension_output(self) -> float:
        return self.extension_motor.getAppliedOutput()
