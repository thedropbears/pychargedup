from rev import CANSparkMax
from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import tunable, feedback
import ids
from components.arm import Arm


class Intake:
    intake_speed = tunable(0.5)

    def __init__(self) -> None:
        self.deployed = False
        self.running = False
        self.break_beam = DigitalInput(ids.DioChannels.intake_break_beam_sensor)
        self.motor = CANSparkMax(
            ids.SparkMaxIds.intake_motor, CANSparkMax.MotorType.kBrushless
        )
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(True)
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ids.PhChannels.intake_piston_forward,
            ids.PhChannels.intake_piston_reverse,
        )

    @feedback
    def is_game_piece_present(self) -> bool:
        return not self.break_beam.get()

    def deploy(self) -> None:
        self.deployed = True
        self.running = True

    def deploy_without_running(self):
        self.deployed = True
        self.running = True

    def retract(self) -> None:
        self.running = False
        self.deployed = False

    def execute(self) -> None:
        if self.running:
            self.motor.set(self.intake_speed)
        else:
            self.motor.set(0.0)

        if self.deployed:
            self.piston.set(DoubleSolenoid.Value.kForward)
        else:
            self.piston.set(DoubleSolenoid.Value.kReverse)
