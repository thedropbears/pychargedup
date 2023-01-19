import rev
from ids import CanIds, PcmChannels, PwmPorts
import math
from wpilib import DutyCycleEncoder, Solenoid, PneumaticsModuleType
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from typing import Optional


class Arm:
    # masses used to calculate gravity feedforwards
    ARM_END_MASS = 5  # kg
    ARM_BASE_MASS = 1
    ROTATE_GRAVITY_FEEDFORWARD = 1
    EXTEND_GRAVITY_FEEDFORWARD = 1

    MIN_EXTENSION = 0.7  # meters
    MAX_EXTENSION = 1.3

    ROTATE_GEAR_RATIO = (60 / 25) * (60 / 25) * (70 / 20)
    EXTEND_GEAR_RATIO = (1 / 7) * (math.pi * 0.05)  # converts to meters

    ANGLE_BOUNDARIES = (
        90,
        270,
    )  # a range of angles that the arm cannot turn to. in the case of (90, 270), the arm wouldn't be able to move to 90 degrees, 120 degrees, etc. to 270 degrees
    # TODO: add the real boundaries

    def __init__(self):

        # Create rotation things
        # left motor is main
        self.rotation_motor = rev.CANSparkMax(
            CanIds.Arm.rotation_left, rev.CANSparkMax.MotorType.kBrushless
        )
        self.rotation_encoder = self.rotation_motor.getEncoder()
        self._rotation_motor_right = rev.CANSparkMax(
            CanIds.Arm.rotation_right, rev.CANSparkMax.MotorType.kBrushless
        )
        self._rotation_motor_right.follow(self.rotation_motor, True)
        self._rotation_motor_right.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.rotation_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        # output position = conversion factor * motor rotations
        self.rotation_encoder.setPositionConversionFactor(self.ROTATE_GEAR_RATIO)
        # also go from RPM to RPS
        self.rotation_encoder.setVelocityConversionFactor(self.ROTATE_GEAR_RATIO * 60)

        # TODO: get pid and feedforward values for arm and extension from sysid
        # running the controller on the rio rather than on the motor controller
        # to allow access to the velocity setpoint for feedforward
        self.rotation_controller = ProfiledPIDController(
            5, 0, 0, TrapezoidProfile.Constraints(maxVelocity=3, maxAcceleration=3)
        )
        # ignore 'meters', theres no generic one
        self.rotation_simple_ff = SimpleMotorFeedforwardMeters(0.1, 0.5, 1)

        # Create extension things
        self.extension_motor = rev.CANSparkMax(
            CanIds.Arm.extension, rev.CANSparkMax.MotorType.kBrushless
        )
        self.extension_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.extension_encoder = self.extension_motor.getEncoder()
        self.extension_encoder.setPositionConversionFactor(self.EXTEND_GEAR_RATIO)
        self.extension_encoder.setVelocityConversionFactor(self.EXTEND_GEAR_RATIO * 60)
        # assume retracted starting position
        self.extension_encoder.setPosition(self.MIN_EXTENSION)
        self.extension_controller = ProfiledPIDController(
            1, 0, 0, TrapezoidProfile.Constraints(maxVelocity=2, maxAcceleration=5)
        )
        self.extension_simple_ff = SimpleMotorFeedforwardMeters(0.1, 0.5, 1)

        self.absolute_encoder = DutyCycleEncoder(PwmPorts.arm_abs_encoder)
        self.absolute_encoder.setPositionOffset(0)
        self.brake_solenoid = Solenoid(
            PneumaticsModuleType.CTREPCM, PcmChannels.arm_brake
        )

        self.goal_angle = 0
        self.goal_extension = self.MIN_EXTENSION

    def execute(self) -> None:
        # Calculate extension motor output
        pid_output = self.extension_controller.calculate(self.get_extension())
        setpoint: TrapezoidProfile.State = self.extension_controller.getSetpoint()
        extend_ff = self.calculate_extension_feedforward(setpoint.velocity)
        self.extension_motor.set(pid_output + extend_ff)

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
        # idk if mass if the right word for this
        arm_mass = (
            self.ARM_BASE_MASS * self.MIN_EXTENSION
            + self.ARM_END_MASS * self.get_extension()
        )
        total_arm_torque = arm_mass * 9.8 * math.cos(self.get_angle())
        # TODO: work out proper math for feedforward with motor constant
        rotate_ff_G = self.ROTATE_GRAVITY_FEEDFORWARD * total_arm_torque
        return rotate_ff_G + self.rotation_simple_ff.calculate(
            self.get_arm_speed(), next_speed
        )

    def calculate_extension_feedforward(self, next_speed: float) -> float:
        arm_end_force = self.ARM_END_MASS * 9.8 * math.sin(-self.get_angle())
        extend_ff_G = self.EXTEND_GRAVITY_FEEDFORWARD * arm_end_force
        return extend_ff_G + self.extension_simple_ff.calculate(next_speed)

    def get_angle(self) -> float:
        """Get the position of the arm in in radians, 0 forwards, CCW down"""
        return self.absolute_encoder.getAbsolutePosition()

    def get_arm_speed(self) -> float:
        """Get the speed of the arm in RPS"""
        return self.rotation_encoder.getVelocity()

    def get_extension(self) -> float:
        """Gets the extension length in meters"""
        return self.MIN_EXTENSION + self.extension_encoder.getPosition()

    def set_angle(self, value: float) -> None:
        """Sets a goal angle to go to in radians, 0 forwards, CCW down"""
        self.goal_angle = value

    def set_length(self, value: int) -> None:
        """Sets a goal length to go to in meters"""
        self.goal_extension = value

    def at_goal_angle(self, allowable_error: Optional[float] = None) -> bool:
        if allowable_error is None:
            allowable_error = math.radians(5)
        return abs(self.get_angle() - self.goal_angle) < allowable_error

    def at_goal_extension(self, allowable_error=0.02) -> bool:
        return abs(self.get_extension() - self.goal_extension) < allowable_error

    def is_angle_still(self, allowable_speed=0.01) -> bool:
        """Allowable speed in RPS"""
        return abs(self.get_arm_speed()) < allowable_speed

    def brake(self):
        self.brake_solenoid.set(True)

    def unbrake(self):
        self.brake_solenoid.set(False)

    def get_target(self, x: float, y: float) -> tuple[float | None, float | None]:
        """y (1.3m)
                        |
                        |
        (-1.3m) --------o------- x (1.3m)
                        |
                        |
                        y (-1.3m)

        o is the center of the arm

        x: the x position to move the arm to
        y: the y position to move the arm to

        returns the (radians needed to get to (x,y), arm length needed to get to (x,y))
        x and y are in meters
        (None, None) return values mean that the position can't be reached
        """

        # first, get the arm length needed
        arm_extension: float = math.sqrt(x**2 + y**2)

        # required extension
        required_extension = arm_extension - self.get_extension()

        if (arm_extension < self.MIN_EXTENSION) or (arm_extension > self.MAX_EXTENSION):
            # the arm can't reach that far, so return none values
            return (None, None)

        # then, get the angle needed from the origin
        angle_from_origin: float = math.atan2(x, y)

        # if the arm can't move to angle_from_origin, return none values
        if (angle_from_origin > self.ANGLE_BOUNDARIES[0]) and (
            angle_from_origin < self.ANGLE_BOUNDARIES[1]
        ):
            return (None, None)

        # get the amount of radians needed to turn
        radians: float = angle_from_origin + self.get_angle()

        # we dont need to make >360 degree turns and we dont need to be exactly precise so modulo works fine.
        # however, negative numbers might cause problems so we get the absolute value and then multiply it by -1 if its negative (or 1 if not)
        if radians > 1:
            radians = (1 if (radians > 0) else -1) * (abs(radians) % 1)

        return (radians, required_extension)

    def set_target(self, x: float, y: float) -> None:
        """Set a position in terms of x and y to move the arm to.
        x: the x position to move the arm to
        y: the y position to move the arm to"""
        target = self.get_target(x, y)

        if target[0] is None:
            return  # don't allow if the arm can't reach

        self.set_angle(target[0])
        self.set_length(int(target[1]))
