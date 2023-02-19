from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import feedback
import time

import ids


class Gripper:
    CLOSE_TIME_THERESHOLD: float = 0.5
    OPEN_TIME_THERESHOLD: float = 0.5

    def __init__(self) -> None:
        self.opened = False
        self.last_opened = self.opened
        self.change_time = time.monotonic()
        self.wants_to_close = False

        self.solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ids.PhChannels.gripper_solenoid_forward,
            ids.PhChannels.gripper_solenoid_reverse,
        )

        self.game_piece_switch = DigitalInput(ids.DioChannels.gripper_game_piece_switch)
        self.set_solenoid = False

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

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
        if self.wants_to_close and self.game_piece_in_reach():
            self.opened = False

        if self.opened != self.last_opened:
            self.change_time = time.monotonic()
        self.last_opened = self.opened

        if self.set_solenoid:
            if self.opened:
                self.solenoid.set(DoubleSolenoid.Value.kReverse)
            else:
                self.solenoid.set(DoubleSolenoid.Value.kForward)

    @feedback
    def game_piece_in_reach(self) -> bool:
        return not self.game_piece_switch.get()
