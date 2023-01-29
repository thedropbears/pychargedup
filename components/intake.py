from rev import CANSparkMax
from wpilib import DoubleSolenoid, Timer
from magicbot import tunable, StateMachine, state, default_state, feedback


class Intake(StateMachine):
    intake_motor: CANSparkMax
    intake_piston: DoubleSolenoid
    intake_speed = tunable(0.5)

    def __init__(self) -> None:
        self.deployed = False
        self.last_x_status = False

    @state
    def intake(self) -> None:
        self.intake_piston.set(DoubleSolenoid.Value.kForward)
        self.intake_motor.set(self.intake_speed)

    @state(first=True)
    def retracted(self) -> None:
        self.intake_piston.set(DoubleSolenoid.Value.kReverse)
        self.intake_motor.set(0.0)
    
    def do_intake(self, x_button) -> None:
        if x_button:
            if self.last_x_status == False:
                self.deployed = not self.deployed
            if self.deployed:
                self.next_state("intake")
            else:
                self.next_state("retracted")
                
        if self.last_x_status != x_button:
            self.last_x_status = x_button
        self.engage()