from rev import CANSparkMax
from wpilib import DoubleSolenoid
from magicbot import tunable, StateMachine, state, default_state


class Intake(StateMachine):
    intake_motor: CANSparkMax
    intake_piston: DoubleSolenoid
    intake_speed = tunable(0.5)

    def __init__(self):
        self.deployed = False

    def update_intake(self):
        if self.deployed:
            self.intake_piston.set(DoubleSolenoid.Value.kForward)
            self.intake_motor.set(self.intake_speed)
        else:
            self.intake_piston.set(DoubleSolenoid.Value.kReverse)
            self.intake_motor.set(0.0)

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