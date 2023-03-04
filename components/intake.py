from rev import CANSparkMax
from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import tunable, feedback
import ids
import time


class Intake:
    intake_speed = tunable(0.5)
    CLOSE_TIME_THRESHOLD: float = 0.5
    OPEN_TIME_THRESHOLD: float = 0.5

    def __init__(self) -> None:
        self.deployed = False
        self._last_deployed = False
        self.change_time = time.monotonic()
        self.running = False
        self.break_beam = DigitalInput(ids.DioChannels.intake_break_beam_sensor)
        self.motor = CANSparkMax(
            ids.SparkMaxIds.intake_motor, CANSparkMax.MotorType.kBrushless
        )
        self.motor.restoreFactoryDefaults()
        self.motor.setSmartCurrentLimit(20)
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
        self.running = False

    def retract(self) -> None:
        self.running = False
        self.deployed = False

    @feedback
    def is_fully_retracted(self) -> bool:
        # has been in same state for some time, current state is closed and wasn't only just closed
        return (
            (time.monotonic() - self.change_time) >= Intake.CLOSE_TIME_THRESHOLD
        ) and not (self._last_deployed or self.deployed)

    @feedback
    def is_fully_deployed(self) -> bool:
        return (
            ((time.monotonic() - self.change_time) >= Intake.OPEN_TIME_THRESHOLD)
            and self.deployed
            and self._last_deployed
        )

    def execute(self) -> None:
        if self.deployed != self._last_deployed:
            self.change_time = time.monotonic()
        self._last_deployed = self.deployed

        if self.running:
            self.motor.set(self.intake_speed)
        else:
            self.motor.set(0.0)

        if self.deployed:
            self.piston.set(DoubleSolenoid.Value.kForward)
        else:
            self.piston.set(DoubleSolenoid.Value.kReverse)
