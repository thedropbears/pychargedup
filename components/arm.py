from magicbot import feedback, tunable
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
from ids import CanIds, PcmChannels
import math
from wpilib import Solenoid, PneumaticsModuleType
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from utilities.functions import clamp


class Arm:
    # masses used to calculate gravity feedforwards
    ARM_END_MASS = 5  # kg
    ARM_BASE_MASS = 1
    ROTATE_GRAVITY_FEEDFORWARD = 1
    EXTEND_GRAVITY_FEEDFORWARD = 1

    MIN_EXTENSION = 0.7  # meters
    MAX_EXTENSION = 1.3

    ROTATE_GEAR_RATIO = (60 / 25) * (60 / 25) * (70 / 20)
    SPOOL_DIAMETER = 0.05
    EXTEND_OUTPUT_RATIO = (1 / 7) * (math.pi * SPOOL_DIAMETER)  # converts to meters

    # Angle soft limits
    MIN_ANGLE = math.radians(-230)
    MAX_ANGLE = math.radians(70)

    # how far either side of vertical will the arm retract if it is in
    # too avoid exceeding the max hieght
    UPRIGHT_ANGLE = math.radians(20)

    goal_angle = tunable(0.0)
    goal_extension = tunable(MIN_EXTENSION)

    def __init__(self):
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
        # ignore 'meters', theres no generic one
        self.rotation_simple_ff = SimpleMotorFeedforwardMeters(0.1, 0.5, 1)

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
        is_going_over or is_currently_up
        # actual_extension_goal = self.MIN_EXTENSION if should_retract else self.goal_extension
        actual_extension_goal = self.goal_extension

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
        # Calculate gravity feedforwards for rotation and extension
        # idk if mass is the right word for this
        arm_mass = (
            self.ARM_BASE_MASS * self.MIN_EXTENSION
            + self.ARM_END_MASS * self.get_extension()
        )
        total_arm_torque = arm_mass * 9.8 * math.cos(self.get_angle())
        # TODO: work out proper math for feedforward with motor constant
        rotate_ff_G = self.ROTATE_GRAVITY_FEEDFORWARD * total_arm_torque
        return rotate_ff_G + self.rotation_simple_ff.calculate(
            self.get_arm_speed(), next_speed, 0.02
        )

    def calculate_extension_feedforward(self, next_speed) -> float:
        arm_end_force = self.ARM_END_MASS * 9.8 * math.sin(-self.get_angle())
        extend_ff_G = self.EXTEND_GRAVITY_FEEDFORWARD * arm_end_force
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
        """Get the speed of the arm in RPS"""
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
        """Is the arm currently not moving, allowable speed is in RPS"""
        return abs(self.get_arm_speed()) < allowable_speed

    def brake(self):
        self.brake_solenoid.set(True)

    def unbrake(self):
        self.brake_solenoid.set(False)

    def set_target(self, x: float, z: float) -> None:
        """Set a position in terms of x and y to move the arm to.
        x: the x position to move the arm to
        z: the z position to move the arm to
        """
        *target, reachable = self.get_arm_from_target(x, z)

        self.set_angle(target[0])
        self.set_length(target[1])

    def get_arm_from_target(self, x: float, z: float) -> tuple[float, float, bool]:
        """z (1.3m)
                        |
                        |
        (-1.3m) --------o------- x (1.3m)
                        |
                        |
                        z (-1.3m)

        o is the center of the arm

        x, z: the forward (x) and vertical (z) position to move the arm to in meters reletive
            to the center of the arm

        returns (
            radians needed to get to (x,z),
            arm length needed to get to (x,z)
            If the position is reachable
        )
        If the position is not reachable it clamps it to something that is and gives False
        """
        reachable = True
        # first, get the arm length needed
        arm_extension = math.sqrt(x**2 + z**2)
        if arm_extension < self.MIN_EXTENSION or arm_extension > self.MAX_EXTENSION:
            reachable = False
            arm_extension = clamp(arm_extension, self.MIN_EXTENSION, self.MAX_EXTENSION)

        # then, get the angle needed from the origin
        arm_angle = math.atan2(z, x)

        # if the arm can't move to angle_from_origin, return none values
        if arm_angle > self.MAX_ANGLE or arm_angle < self.MIN_ANGLE:
            reachable = False
            arm_angle = clamp(arm_angle, self.MIN_ANGLE, self.MAX_ANGLE)

        return (arm_angle, arm_extension, reachable)

    def on_enable(self):
        self.extension_controller.reset(self.get_extension())
        self.rotation_controller.reset(self.get_angle())
