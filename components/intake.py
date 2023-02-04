from rev import CANSparkMax
from wpilib import DoubleSolenoid, PneumaticsModuleType
from magicbot import tunable
import ids


class Intake:
    intake_speed = tunable(0.5)

    def __init__(self):
        self.deployed = False
        self.motor = CANSparkMax(
            ids.CanIds.Intake.intake_motor, CANSparkMax.MotorType.kBrushless
        )
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.Intake.intake_piston_forward,
            ids.PcmChannels.Intake.intake_piston_reverse,
        )

    def execute(self):
        if self.deployed:
            self.piston.set(DoubleSolenoid.Value.kForward)
            self.motor.set(self.intake_speed)
        else:
            self.piston.set(DoubleSolenoid.Value.kReverse)
            self.motor.set(0.0)

    def deploy(self):
        self.deployed = True

    def retract(self):
        self.deployed = False
