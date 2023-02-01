import rev
import wpilib


class SparkMaxEncoderWrapper:
    """Wraps an spark max relative encoder and allows you to set the velocity in the sim"""

    def __init__(self, motor: rev.CANSparkMax) -> None:
        self.real_encoder = motor.getEncoder()
        self.sim_position = 0.0
        self.sim_velocity = 0.0

    def getPosition(self) -> float:
        if wpilib.RobotBase.isReal():
            return self.real_encoder.getPosition()
        else:
            return self.sim_position

    def getVelocity(self) -> float:
        if wpilib.RobotBase.isReal():
            return self.real_encoder.getVelocity()
        else:
            return self.sim_velocity

    def set_position(self, position: float) -> None:
        self.real_encoder.setPosition(position)

    def set_velocity(self, velocity: float) -> None:
        self.sim_velocity = velocity
