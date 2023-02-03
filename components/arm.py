from magicbot import feedback, tunable
from rev import CANSparkMax
import wpilib
from ids import CanIds, PcmChannels, DioChannels
import math
from wpilib import (
    Solenoid,
    PneumaticsModuleType,
    DutyCycleEncoder,
    SendableChooser,
    SmartDashboard,
)
from wpimath.controller import (
    ProfiledPIDController,
    ArmFeedforward,
    SimpleMotorFeedforwardMeters,
)
from wpimath.trajectory import TrapezoidProfile
from utilities.functions import clamp

MIN_EXTENSION = 0.7  # meters
MAX_EXTENSION = 1.3

# Angle soft limits
MIN_ANGLE = math.radians(-230)
MAX_ANGLE = math.radians(70)


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


class Setpoints:
    PICKUP_CONE = Setpoint(-math.pi, MIN_EXTENSION + 0.05)
    HANDOFF = Setpoint(MAX_ANGLE, MIN_EXTENSION + 0.1)
    SCORE_CONE_MID = Setpoint.fromCartesian(-0.80, 0.12)
    SCORE_CUBE_MID = Setpoint.fromCartesian(-0.80, -0.20)
    SCORE_CONE_HIGH = Setpoint.fromCartesian(-1.22, 0.42)
    SCORE_CUBE_HIGH = Setpoint.fromCartesian(-1.22, 0.10)


class Arm:
    # height of the center of rotation off the ground
    HEIGHT = 1

    ROTATE_GEAR_RATIO = (74 / 14) * (82 / 26) * (42 / 18)
    SPOOL_DIAMETER = 0.05  # cm
    EXTEND_GEAR_RATIO = 7 / 1
    # converts from motor rotations to meters
    EXTEND_OUTPUT_RATIO = 1 / EXTEND_GEAR_RATIO * math.pi * SPOOL_DIAMETER
    EXTEND_GRAVITY_FEEDFORWARD = 0
    ROTATE_GRAVITY_FEEDFORWARDS = 6

    # how far either side of vertical will the arm retract if it is in
    # too avoid exceeding the max hieght
    UPRIGHT_ANGLE = math.radians(20)

    goal_angle = tunable(0.0)
    goal_extension = tunable(MIN_EXTENSION)
    # automatically retract the extension when rotating overhead
    do_auto_retract = tunable(True)

    def __init__(self) -> None:
        # Create rotation things
        self.rotation_motor = CANSparkMax(
            CanIds.Arm.rotation_main, CANSparkMax.MotorType.kBrushless
        )
        self.rotation_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.rotation_motor.setInverted(False)
        # setup second motor to follow first
        self._rotation_motor_follower = CANSparkMax(
            CanIds.Arm.rotation_follower, CANSparkMax.MotorType.kBrushless
        )
        self._rotation_motor_follower.follow(self.rotation_motor, invert=True)
        self._rotation_motor_follower.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self._rotation_motor_follower.setInverted(False)
        self.relative_encoder = self.rotation_motor.getEncoder()
        self.relative_encoder.setPositionConversionFactor(1 / self.ROTATE_GEAR_RATIO)
        self.relative_encoder.setVelocityConversionFactor(
            1 / self.ROTATE_GEAR_RATIO / 60
        )

        self.absolute_encoder = DutyCycleEncoder(DioChannels.Arm.absolute_encoder)
        self.absolute_encoder.setDistancePerRotation(math.tau)
        self.absolute_encoder.setPositionOffset(0)

        # TODO: get pid and feedforward values for arm and extension from sysid
        # running the controller on the rio rather than on the motor controller
        # to allow access to the velocity setpoint for feedforward
        rotation_constraints = TrapezoidProfile.Constraints(
            maxVelocity=4, maxAcceleration=5
        )
        self.rotation_controller = ProfiledPIDController(24, 0, 0, rotation_constraints)
        self.rotation_ff = ArmFeedforward(
            kS=0, kG=-self.ROTATE_GRAVITY_FEEDFORWARDS, kV=0, kA=12
        )

        # Create extension things
        self.extension_motor = CANSparkMax(
            CanIds.Arm.extension, CANSparkMax.MotorType.kBrushless
        )
        self.extension_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.extension_motor.setInverted(False)
        self.extension_encoder = self.extension_motor.getEncoder()
        self.extension_encoder.setPositionConversionFactor(1 / self.EXTEND_OUTPUT_RATIO)
        self.extension_encoder.setVelocityConversionFactor(
            1 / self.EXTEND_OUTPUT_RATIO / 60
        )
        # assume retracted starting position
        self.extension_encoder.setPosition(MIN_EXTENSION)
        self.extension_controller = ProfiledPIDController(
            48, 0, 0, TrapezoidProfile.Constraints(maxVelocity=3.0, maxAcceleration=6)
        )
        self.extension_simple_ff = SimpleMotorFeedforwardMeters(kS=0, kV=2, kA=2)

        self.brake_solenoid = Solenoid(
            PneumaticsModuleType.CTREPCM, PcmChannels.arm_brake
        )
        self.chooser = SendableChooser()
        self.chooser.addOption("Score Cone High", Setpoints.SCORE_CONE_HIGH)
        self.chooser.addOption("Score Cube High", Setpoints.SCORE_CUBE_HIGH)
        self.chooser.addOption("Score Cone Mid", Setpoints.SCORE_CONE_MID)
        self.chooser.addOption("Score Cube mid", Setpoints.SCORE_CUBE_MID)
        self.chooser.addOption("Handoff", Setpoints.HANDOFF)
        self.chooser.setDefaultOption("Handoff", Setpoints.HANDOFF)
        SmartDashboard.putData("Arm Setpoint Chooser", self.chooser)
        self.last_selection = Setpoint(0, 0)

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
        wpilib.SmartDashboard.putData("Arm sim", self.arm_mech2d)

    def setup(self) -> None:
        self.set_setpoint(Setpoints.SCORE_CONE_HIGH)

    def execute(self) -> None:
        extend_state: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        rotate_state = self.rotation_controller.getSetpoint()

        # Update display
        self.arm_ligament.setAngle(-math.degrees(self.get_angle()))
        self.arm_goal_ligament.setAngle(-math.degrees(rotate_state.position))
        self.arm_extend_ligament.setLength(self.get_extension() - MIN_EXTENSION)
        self.arm_extend_goal_ligament.setLength(extend_state.position - MIN_EXTENSION)

        setpoint: Setpoint = self.chooser.getSelected()  # type: ignore
        if setpoint != self.last_selection:
            self.set_setpoint(setpoint)
        self.last_selection = setpoint

        extension_goal = self.get_max_extension()
        # Calculate extension motor output
        pid_output = self.extension_controller.calculate(
            self.get_extension(), extension_goal
        )
        extension_ff = self.calculate_extension_feedforward(extend_state.velocity)
        self.extension_motor.setVoltage(pid_output + extension_ff)

        if self.at_goal_angle() and self.is_angle_still():
            self.brake()
            self.rotation_motor.set(0)
            return
        else:
            self.unbrake()

        # Calculate rotation motor output
        pid_output = self.rotation_controller.calculate(
            self.get_angle(), self.goal_angle
        )
        rotation_ff = self.calculate_rotation_feedforwards(rotate_state.velocity)
        self.rotation_motor.setVoltage(pid_output + rotation_ff)

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

    def calculate_rotation_feedforwards(self, next_speed: float) -> float:
        """Calculate feedforwards voltage.
        next_speed: speed in rps"""
        accel = next_speed - self.get_arm_speed()
        return self.rotation_ff.calculate(self.get_angle(), self.get_arm_speed(), accel)

    def calculate_extension_feedforward(self, next_speed: float) -> float:
        extend_ff_G = self.EXTEND_GRAVITY_FEEDFORWARD * math.sin(-self.get_angle())
        accel = next_speed - self.get_extension_speed()
        extend_ff_simple = self.extension_simple_ff.calculate(next_speed, accel)
        return extend_ff_G + extend_ff_simple

    @feedback
    def get_angle(self) -> float:
        """Get the position of the arm in in radians, 0 forwards, CCW down"""
        return self.absolute_encoder.getDistance()

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

    def set_angle(self, value: float) -> None:
        """Sets a goal angle to go to in radians, 0 forwards, CCW down"""
        self.goal_angle = clamp(value, MIN_ANGLE, MAX_ANGLE)

    def set_length(self, value: float) -> None:
        """Sets a goal length to go to in meters"""
        self.goal_extension = clamp(value, MIN_EXTENSION, MAX_EXTENSION)

    def set_setpoint(self, value: Setpoint) -> None:
        self.set_length(value.extension)
        self.set_angle(value.angle)

    DEFAULT_ALLOWABLE_ANGLE_ERROR = math.radians(2)

    def at_goal_angle(
        self, allowable_error: float = DEFAULT_ALLOWABLE_ANGLE_ERROR
    ) -> bool:
        return abs(self.get_angle() - self.goal_angle) < allowable_error

    def at_goal_extension(self, allowable_error=0.05) -> bool:
        return abs(self.get_extension() - self.goal_extension) < allowable_error

    def is_angle_still(self, allowable_speed=0.01) -> bool:
        """Is the arm currently not moving, allowable speed is in Rotations/s"""
        return abs(self.get_arm_speed()) < allowable_speed

    def brake(self) -> None:
        self.brake_solenoid.set(False)

    def unbrake(self) -> None:
        self.brake_solenoid.set(True)

    def on_enable(self) -> None:
        self.extension_controller.reset(self.get_extension())
        self.rotation_controller.reset(self.get_angle())
