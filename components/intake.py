from rev import CANSparkMax
from wpilib import DoubleSolenoid


class intake:
    intake_motor: CANSparkMax
    intake_piston: DoubleSolenoid

    def __init__(self):
        self.deployed = False

    def execute(self):
        if self.deployed:
            self.intake_piston.set(DoubleSolenoid.Value.kForward)
            self.intake_motor.set(1.0)
        else:
            self.intake_piston.set(DoubleSolenoid.Value.kReverse)
            self.intake_motor.set(0.0)

    def deploy(self):
        self.deployed = True

    def retract(self):
        self.deployed = False
