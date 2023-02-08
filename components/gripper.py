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

        self.solenoid = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.gripper_solenoid_forward,
            ids.PcmChannels.gripper_solenoid_reverse,
        )

        self.game_piece_switch = DigitalInput(ids.DioChannels.gripper_game_piece_switch)

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    @feedback
    def get_full_closed(self) -> bool:
        return (
            (time.monotonic() - self.change_time) >= Gripper.CLOSE_TIME_THERESHOLD
        ) and (not self.opened)

    @feedback
    def get_full_open(self) -> bool:
        return (
            (time.monotonic() - self.change_time) >= Gripper.OPEN_TIME_THERESHOLD
        ) and self.opened

    def execute(self) -> None:
        if self.opened != self.last_opened:
            self.change_time = time.monotonic()
            self.last_opened = self.opened

        if self.opened:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.solenoid.set(DoubleSolenoid.Value.kForward)

    @feedback
    def game_piece_in_reach(self) -> bool:
        return not self.game_piece_switch.get()
