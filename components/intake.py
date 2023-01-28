from rev import CANSparkMax
from wpilib import DoubleSolenoid
from magicbot import tunable


class Intake:
    intake_motor: CANSparkMax
    intake_piston: DoubleSolenoid
    intake_speed = tunable(0.5)

    def __init__(self):
        self.deployed = False

    def execute(self):
        if self.deployed:
            self.intake_piston.set(DoubleSolenoid.Value.kForward)
            self.intake_motor.set(self.intake_speed)
        else:
            self.intake_piston.set(DoubleSolenoid.Value.kReverse)
            self.intake_motor.set(0.0)

    def deploy(self):
        self.deployed = True

    def retract(self):
        self.deployed = False
