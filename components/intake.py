from rev import CANSparkMax
from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import tunable, StateMachine, state
import ids


class Intake(StateMachine):
    intake_speed = tunable(0.5)

    def __init__(self) -> None:
        self.deployed = False
        self.last_x_status = False
        self.break_beam = DigitalInput(0)
        self.motor = CANSparkMax(
            ids.CanIds.Intake.intake_motor, CANSparkMax.MotorType.kBrushless
        )
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.Intake.intake_piston_forward,
            ids.PcmChannels.Intake.intake_piston_reverse,
        )

    @state
    def intake(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kForward)
        self.motor.set(self.intake_speed)

    @state(first=True)
    def retracted(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kReverse)
        self.motor.set(0.0)

    def do_intake(self, x_button) -> None:
        if x_button and self.last_x_status is False:
            self.deployed = not self.deployed

        if self.deployed and self.current_state != "intake":
            self.next_state("intake")
        elif not self.deployed and self.current_state != "retracted":
            self.next_state("retracted")

        if not self.break_beam.get() and self.current_state == "intake":
            self.deployed = False

        if self.last_x_status != x_button:
            self.last_x_status = x_button
        self.engage()
