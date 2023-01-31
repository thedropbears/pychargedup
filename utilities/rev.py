import rev
import wpilib


class SparkMaxAbsoluteEncoderWrapper:
    def __init__(self, motor: rev.CANSparkMax) -> None:
        self.real_encoder = motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.sim_position = 0
        self.sim_velocity = 0

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
        self.sim_position = position

    def set_velocity(self, velocity: float) -> None:
        self.sim_velocity = velocity
