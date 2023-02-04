from rev import CANSparkMax
from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import tunable
import ids


class Intake:
    intake_speed = tunable(0.5)

    def __init__(self) -> None:
        self.deployed = False
        self.break_beam = DigitalInput(ids.DioChannels.Intake.break_beam_sensor)
        self.motor = CANSparkMax(
            ids.CanIds.Intake.intake_motor, CANSparkMax.MotorType.kBrushless
        )
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.Intake.intake_piston_forward,
            ids.PcmChannels.Intake.intake_piston_reverse,
        )

    def is_game_piece_present(self) -> bool:
        return not self.break_beam.get()

    def deploy(self) -> None:
        self.deployed = True

    def retract(self) -> None:
        self.deployed = False

    def execute(self) -> None:
        if self.deployed:
            self.piston.set(DoubleSolenoid.Value.kForward)
            self.motor.set(self.intake_speed)
        else:
            self.piston.set(DoubleSolenoid.Value.kReverse)
            self.motor.set(0.0)
        if self.is_game_piece_present():
            self.deployed = False
