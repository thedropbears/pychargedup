import rev
import ids
import math

from wpimath.geometry import Translation2d

class Arm:

    GRAVITY_FEED_FORWARD = 1   ;   GRAVITY_EXTENSION_FEED_FORWARD = 1  ;   MAX_EXTENSION = 1
    extended = 0               ;   ref = 0

    def __init__(self):
        self.motor = rev.CANSparkMax(ids.CanIds.Arm.rotation_1, rev.CANSparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.controller = self.motor.getPIDController()
        self.controller.setP(1)
        self.controller.setI(0)
        self.controller.setD(0)
        self.encoder.setPositionConversionFactor(100000001)

    def setup(self) -> None:
        self.controller.setSmartMotionAccelStrategy(rev.SparkMaxPIDController.AccelStrategy.kTrapezoidal)

    def get_position(self) -> float:
        """Get the position of the motor in rotations"""
        return self.encoder.getPosition()

    def get_position_in_degrees(self) -> float:
        degrees = self.get_position()
        if (degrees == 360 or degrees == -360):
          return 0
        return degrees

    def get_reference(self) -> float:
        return self.ref

    def get_extended(self) -> int:
        return self.extended

    def get_velocity(self) -> int:
        """Get the velocity of the motor in RPM"""
        return self.encoder.getVelocity()

    def execute(self) -> None:
        gravity_feed_forward = (self.GRAVITY_FEED_FORWARD *
                                math.cos(self.get_position_in_degrees()) *
                                self.GRAVITY_EXTENSION_FEED_FORWARD *
                                self.extended/self.MAX_EXTENSION)
        self.controller.setReference(self.ref, rev.CANSparkMaxLowLevel.ControlType.kPosition, 0, gravity_feed_forward)

    def set_reference(self, value: float = 1) -> None:
        self.ref = value

    def set_extended(self, extended: int) -> None:
        self.extended = extended
