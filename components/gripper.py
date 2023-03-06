from wpilib import DoubleSolenoid, PneumaticsModuleType, DigitalInput
from magicbot import feedback
import time
from utilities.game import GamePiece
import ids


class Gripper:
    CLOSE_TIME_THERESHOLD: float = 0.5
    OPEN_TIME_THERESHOLD: float = 0.5

    def __init__(self) -> None:
        self.opened_gripper = False
        self.opened_flapper = False
        self.last_opened = self.opened_gripper
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
        """Open both the gripper and the flapper"""
        self.open_gripper()
        self.open_flapper()

    def open_gripper(self) -> None:
        self.opened_gripper = True

    def open_flapper(self) -> None:
        self.opened_flapper = True

    def close(self, on: GamePiece = GamePiece.BOTH) -> None:
        """Close both the gripper and the flapper"""
        self.close_gripper(on)
        self.close_flapper()

    def close_gripper(self, on: GamePiece = GamePiece.BOTH) -> None:
        self.opened_gripper = False
        self.holding = on

    def close_flapper(self) -> None:
        self.opened_flapper = False

    @feedback
    def get_full_closed(self) -> bool:
        # has been in same state for some time, current state is closed and wasn't only just closed
        return (
            (time.monotonic() - self.change_time) >= Gripper.CLOSE_TIME_THERESHOLD
        ) and self.last_opened is self.opened_gripper is False

    @feedback
    def get_full_open(self) -> bool:
        return (
            (time.monotonic() - self.change_time) >= Gripper.OPEN_TIME_THERESHOLD
        ) and self.opened_gripper is self.last_opened is True

    def is_closing(self) -> bool:
        return (not self.opened_gripper) and not self.get_full_closed()

    def is_opening(self) -> bool:
        return self.opened_gripper and not self.get_full_open()

    def execute(self) -> None:
        if self.opened_gripper != self.last_opened:
            self.change_time = time.monotonic()
        self.last_opened = self.opened_gripper

        if self.opened_gripper:
            self.gripper_solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.gripper_solenoid.set(DoubleSolenoid.Value.kForward)

        if self.opened_flapper:
            self.flapper_solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.flapper_solenoid.set(DoubleSolenoid.Value.kForward)

    def get_current_piece(self) -> GamePiece:
        if self.opened_gripper:
            return GamePiece.NONE
        return self.holding
    
    @feedback
    def get_current_piece_as_int(self) -> int:
        piece = self.get_current_piece()
        if piece == GamePiece.NONE:
            return 0
        if piece == GamePiece.CONE:
            return 1
        if piece == GamePiece.CUBE:
            return 2
        return -1

    @feedback
    def cube_present(self) -> bool:
        return self.get_full_open() and self.cube_break_beam_broken()

    @feedback
    def cube_break_beam_broken(self):
        return not self.cube_break_beam.get()
