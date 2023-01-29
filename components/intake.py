from rev import CANSparkMax
from wpilib import DoubleSolenoid, PneumaticsModuleType
from magicbot import tunable, StateMachine, state, default_state
import ids


class Intake(StateMachine):
    intake_motor: CANSparkMax
    intake_piston: DoubleSolenoid
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

    def update_intake(self):
        if self.deployed:
            self.piston.set(DoubleSolenoid.Value.kForward)
            self.motor.set(self.intake_speed)
        else:
            self.piston.set(DoubleSolenoid.Value.kReverse)
            self.motor.set(0.0)

    @state
    def intaking(self):
        self.deployed = True
        self.update_intake()

    @default_state
    def retracted(self):
        self.deployed = False
        self.update_intake()
    
    def do_intake(self):
        self.next_state("intaking")
        self.engage()