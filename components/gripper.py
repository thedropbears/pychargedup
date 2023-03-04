from wpilib import DoubleSolenoid, PneumaticsModuleType, DigitalInput
from magicbot import feedback
import time
from utilities.game import GamePiece
import ids


class Gripper:
    CLOSE_TIME_THERESHOLD: float = 0.5
    OPEN_TIME_THERESHOLD: float = 0.5

    def __init__(self) -> None:
        self.opened = False
        self.last_opened = self.opened
        self.change_time = time.monotonic()

        self.gripper_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ids.PhChannels.gripper_solenoid_forward,
            ids.PhChannels.gripper_solenoid_reverse,
        )
        self.flapper_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ids.PhChannels.flapper_solenoid_forward,
            ids.PhChannels.flapper_solenoid_reverse,
        )

        self.cube_break_beam = DigitalInput(ids.DioChannels.gripper_cube_break_beam)

        self.holding = GamePiece.NONE

    def open(self) -> None:
        self.opened = True

    def close(self, on: GamePiece = GamePiece.BOTH) -> None:
        self.opened = False
        self.holding = on

    @feedback
    def get_full_closed(self) -> bool:
        # has been in same state for some time, current state is closed and wasn't only just closed
        return (
            (time.monotonic() - self.change_time) >= Gripper.CLOSE_TIME_THERESHOLD
        ) and self.last_opened is self.opened is False

    @feedback
    def get_full_open(self) -> bool:
        return (
            (time.monotonic() - self.change_time) >= Gripper.OPEN_TIME_THERESHOLD
        ) and self.opened is self.last_opened is True

    def is_closing(self) -> bool:
        return (not self.opened) and not self.get_full_closed()

    def is_opening(self) -> bool:
        return self.opened and not self.get_full_open()

    def execute(self) -> None:
        if self.opened != self.last_opened:
            self.change_time = time.monotonic()
        self.last_opened = self.opened

        if self.opened:
            self.gripper_solenoid.set(DoubleSolenoid.Value.kReverse)
            self.flapper_solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.gripper_solenoid.set(DoubleSolenoid.Value.kForward)
            self.flapper_solenoid.set(DoubleSolenoid.Value.kForward)

    def get_current_piece(self) -> GamePiece:
        if self.opened:
            return GamePiece.NONE
        return self.holding
