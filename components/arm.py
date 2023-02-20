from magicbot import feedback, tunable
import wpilib
from ids import SparkMaxIds, PcmChannels, DioChannels
import math
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

MIN_EXTENSION = 0.9  # meters
MAX_EXTENSION = 1.3

# Angle soft limits
MIN_ANGLE = math.radians(-230)
MAX_ANGLE = 1


class Setpoint:
    # arm angle in radians
    # angle of 0 points towards positive X, at the intake
    # positive counterclockwise when looking at the left side of the robot
    angle: float
    # extension in meters, length from center of rotation to center of where pieces are held in the end effector
    extension: float

    def __init__(self, angle: float, extension: float) -> None:
        self.extension = clamp(extension, MIN_EXTENSION, MAX_EXTENSION)
        if angle > MAX_ANGLE:  # and (self.angle - math.tau) < Arm.MIN_ANGLE:
            angle -= math.tau
        self.angle = clamp(angle, MIN_ANGLE, MAX_ANGLE)

        if self.angle != angle or self.extension != extension:
            print(
                "SETPOINT WAS CLAMPED", (angle, extension), (self.angle, self.extension)
            )

    @classmethod
    def fromCartesian(cls, x: float, z: float) -> "Setpoint":
        """Sets a cartesian goal reletive to the arms axle"""
        return cls(math.atan2(-z, x), math.hypot(x, z))

    def toCartesian(self) -> tuple[float, float]:
        return self.extension * math.cos(self.angle), self.extension * math.sin(
            self.angle
        )


class Setpoints:
    PICKUP_CONE = Setpoint(-3.05, 1.0)
    HANDOFF = Setpoint(0.8, MIN_EXTENSION)
    STOW = Setpoint(0.35, MIN_EXTENSION)
    START = Setpoint(-math.radians(30), MIN_EXTENSION)
    SCORE_CONE_MID = Setpoint(-2.8, 0.89)
    SCORE_CUBE_MID = Setpoint(-3.2, 0.89)
    SCORE_CONE_HIGH = Setpoint(-2.89, 1.17)
    SCORE_CUBE_HIGH = Setpoint(-3, 1.17)

    UPRIGHT = Setpoint(-math.pi / 2, MIN_EXTENSION + 0.1)
    FORWARDS = Setpoint(0, MIN_EXTENSION)
    BACKWARDS = Setpoint(-math.pi, MIN_EXTENSION)


class Arm:
    # height of the center of rotation off the ground
    HEIGHT = 1

    ROTATE_GEAR_RATIO = (74 / 14) * (82 / 26) * (42 / 18)
    SPOOL_CIRCUMFERENCE = 42 * 0.005  # 42t x 5mm
    EXTEND_GEAR_RATIO = (7 / 1) * (34 / 18)
    # converts from motor rotations to meters
    EXTEND_OUTPUT_RATIO = SPOOL_CIRCUMFERENCE / EXTEND_GEAR_RATIO
    EXTEND_GRAVITY_FEEDFORWARD = 0
    ROTATE_GRAVITY_FEEDFORWARDS = 2.5

    ARM_ENCODER_ANGLE_OFFSET = 0.325  # rotations 0-1
    # how far either side of vertical will the arm retract if it is in
    # too avoid exceeding the max hieght
    UPRIGHT_ANGLE = math.radians(20)

    goal_angle = tunable(0.0)
    goal_extension = tunable(MIN_EXTENSION)
    # automatically retract the extension when rotating overhead
    do_auto_retract = tunable(True)

    control_loop_wait_time: float

    def __init__(self) -> None:
        # Create rotation things
        self.rotation_motor = rev.CANSparkMax(
            SparkMaxIds.arm_rotation_main, rev.CANSparkMax.MotorType.kBrushless
        )
        self.rotation_motor.restoreFactoryDefaults()
        self.rotation_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.rotation_motor.setInverted(True)
        # setup second motor to follow first
        self._rotation_motor_follower = rev.CANSparkMax(
            SparkMaxIds.arm_rotation_follower, rev.CANSparkMax.MotorType.kBrushless
        )
        self._rotation_motor_follower.restoreFactoryDefaults()
        self._rotation_motor_follower.follow(self.rotation_motor, invert=False)
        self._rotation_motor_follower.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self._rotation_motor_follower.setInverted(True)
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
            maxVelocity=3, maxAcceleration=2
        )
        self.rotation_controller = ProfiledPIDController(
            10, 0, 0.1, rotation_constraints
        )
        self.rotation_ff = ArmFeedforward(
            kS=0, kG=-self.ROTATE_GRAVITY_FEEDFORWARDS, kV=1, kA=0.1
        )
        self.rotation_last_setpoint_vel = 0

        # Create extension things
        self.extension_motor = rev.CANSparkMax(
            SparkMaxIds.arm_extension, rev.CANSparkMax.MotorType.kBrushless
        )
        self.extension_motor.restoreFactoryDefaults()
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
            30, 0, 0, TrapezoidProfile.Constraints(maxVelocity=2.0, maxAcceleration=4.0)
        )
        self.extension_simple_ff = SimpleMotorFeedforwardMeters(kS=0, kV=2, kA=0.2)
        self.extension_last_setpoint_vel = 0

        self.brake_solenoid = Solenoid(
            PneumaticsModuleType.CTREPCM, PcmChannels.arm_brake
        )

        # Create arm display
        self.arm_mech2d = wpilib.Mechanism2d(5, 3)
        arm_pivot = self.arm_mech2d.getRoot("ArmPivot", 2.5, self.HEIGHT)
        arm_pivot.appendLigament("ArmTower", self.HEIGHT - 0.05, -90)
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
        self.hall_effector_forward_arm = self.extension_motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self.hall_effector_inner_arm = self.extension_motor.getReverseLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self.homing = False

        wpilib.SmartDashboard.putData("Arm sim", self.arm_mech2d)

        self.voltage_movement = False
        self.arm_velocity = 0

    def setup(self) -> None:
        self.set_length(self.get_extension())

    def execute(self) -> None:
        extend_state: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        rotate_state = self.rotation_controller.getSetpoint()

        # Update display
        self.arm_ligament.setAngle(-math.degrees(self.get_angle()))
        self.arm_goal_ligament.setAngle(-math.degrees(rotate_state.position))
        self.arm_extend_ligament.setLength(self.get_extension() - MIN_EXTENSION)
        self.arm_extend_goal_ligament.setLength(extend_state.position - MIN_EXTENSION)

        # if self.is_extended():
        #     self.extension_encoder.setPosition(MAX_EXTENSION)
        # if self.is_retracted():
        #     self.extension_encoder.setPosition(MIN_EXTENSION)

        if self.homing:
            self.extension_motor.set(-0.2)
            self.rotation_motor.set(0)
            return
        elif self.voltage_movement:
            self.extension_motor.set(self.arm_velocity)
        else:
            extension_goal = self.get_max_extension()
            # Calculate extension motor output
            pid_output = self.extension_controller.calculate(
                self.get_extension(), extension_goal
            )
            self.calculate_extension_feedforward()
            self.extension_motor.setVoltage(pid_output)

        if self.at_goal_angle(math.radians(3)) and self.is_angle_still():
            self.brake()
        if not self.at_goal_angle(math.radians(5)):
            self.unbrake()
        if self.is_braking():
            self.rotation_motor.set(0)
            return

        # Calculate rotation motor output
        pid_output = self.rotation_controller.calculate(
            self.get_angle(), self.goal_angle
        )
        self.calculate_rotation_feedforwards()
        self.rotation_motor.setVoltage(pid_output)

    def get_max_extension(self) -> float:
        """Gets the max extension to not exceed the height limit for the current angle and goal"""
        is_angle_forwards = math.copysign(1, self.get_angle() + (math.pi / 2))
        is_goal_forwards = math.copysign(1, self.goal_angle + (math.pi / 2))
        # if we plan on rotating overhead
        is_going_over = is_angle_forwards != is_goal_forwards
        # if we are currently overhead
        is_currently_up = (
            self.UPRIGHT_ANGLE > (self.get_angle() + math.pi / 2) > -self.UPRIGHT_ANGLE
        )
        should_retract = (is_going_over or is_currently_up) and self.do_auto_retract
        return MIN_EXTENSION if should_retract else self.goal_extension

    def get_near_intake(self) -> bool:
        """Gets if the arm may hit the intake currently"""
        # Assume all setpoints are good
        return not self.at_goal() and self.get_angle() > math.radians(20)

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

    def extend(self, velocity: float):
        self.arm_velocity = velocity
        self.voltage_movement = True

    def retract(self, velocity: float):
        self.arm_velocity = -velocity
        self.voltage_movement = True

    @feedback
    def get_angle(self) -> float:
        """Get the position of the arm in in radians, 0 forwards, CCW down"""
        return self.absolute_encoder.getDistance() + self.runtime_offset

    @feedback
    def get_raw_angle(self) -> float:
        return self.absolute_encoder.get()

    @feedback
    def get_arm_speed(self) -> float:
        """Get the speed of the arm in Rotations/s"""
        # uses the relative encoder beacuse the absolute one dosent report velocity
        return self.relative_encoder.getVelocity()

    @feedback
    def get_extension(self) -> float:
        """Gets the extension length in meters from axle"""
        return self.extension_encoder.getPosition()

    @feedback
    def get_extension_speed(self) -> float:
        """Gets the extension speed in m/s"""
        return self.extension_encoder.getVelocity()

    @feedback
    def is_extended(self) -> bool:
        return self.hall_effector_forward_arm.get()

    def set_at_min_extension(self) -> None:
        """Sets the extension position to the min extension"""
        self.extension_encoder.setPosition(MIN_EXTENSION)

    @feedback
    def is_retracted(self) -> bool:
        return self.hall_effector_inner_arm.get()

    def set_angle(self, value: float) -> None:
        """Sets a goal angle to go to in radians, 0 forwards, CCW down"""
        self.goal_angle = clamp(value, MIN_ANGLE, MAX_ANGLE)

    def set_length(self, value: float) -> None:
        """Sets a goal length to go to in meters"""
        self.goal_extension = clamp(value, MIN_EXTENSION, MAX_EXTENSION)
        self.voltage_movement = False

    def go_to_setpoint(self, value: Setpoint) -> None:
        self.set_length(value.extension)
        self.set_angle(value.angle)

    DEFAULT_ALLOWABLE_ANGLE_ERROR = math.radians(5)

    def at_goal_angle(
        self, allowable_error: float = DEFAULT_ALLOWABLE_ANGLE_ERROR
    ) -> bool:
        return abs(self.get_angle() - self.goal_angle) < allowable_error

    def at_goal_extension(self, allowable_error=0.1) -> bool:
        return abs(self.get_extension() - self.goal_extension) < allowable_error

    def is_angle_still(self, allowable_speed=0.01) -> bool:
        """Is the arm currently not moving, allowable speed is in Rotations/s"""
        return abs(self.get_arm_speed()) < allowable_speed

    def at_goal(self) -> bool:
        return (
            self.at_goal_extension() and self.at_goal_angle() and self.is_angle_still()
        )

    def brake(self) -> None:
        self.brake_solenoid.set(False)

    def unbrake(self) -> None:
        self.brake_solenoid.set(True)

    def is_braking(self) -> bool:
        return not self.brake_solenoid.get()

    def on_enable(self) -> None:
        if self.get_angle() > math.pi / 2:
            self.runtime_offset = -math.tau
        self.reset_controllers()
        self.unbrake()

    def stop(self) -> None:
        self.rotation_motor.set(0)
        self.extension_motor.set(0)
        self.voltage_movement = False

    def reset_controllers(self) -> None:
        self.extension_controller.reset(self.get_extension())
        self.rotation_controller.reset(self.get_angle())
