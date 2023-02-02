import rev
import wpilib


class SparkMaxEncoderWrapper:
    """Wraps an spark max encoder and allows you to set the velocity in the sim"""

    def __init__(self, motor: rev.CANSparkMax, reduction: float) -> None:
        """
        motor: Spark max to get the encoder from
        reduction: value to use for the position convertion, >1 for reductions"""
        self.real_encoder = motor.getEncoder()
        self.real_encoder.setPositionConversionFactor(reduction)
        self.real_encoder.setVelocityConversionFactor(reduction / 60)
        self.sim_position = 0.0
        self.sim_velocity = 0.0

    def getPosition(self) -> float:
        return self.real_encoder.getPosition()

    def getVelocity(self) -> float:
        if wpilib.RobotBase.isReal():
            return self.real_encoder.getVelocity()
        else:
            return self.sim_velocity

    def setPosition(self, position: float) -> None:
        self.real_encoder.setPosition(position)

    def setVelocity(self, velocity: float) -> None:
        self.sim_velocity = velocity
