from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType
from magicbot import feedback
import time

import ids


class Gripper:
    def __init__(self) -> None:
        self.opened = False
        self.close_time = time.monotonic()

        self.solenoid = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.Gripper.gripper_solenoid_forward,
            ids.PcmChannels.Gripper.gripper_solenoid_reverse,
        )

        self.game_piece_switch = DigitalInput(
            ids.DioChannels.Gripper.gripper_game_piece_switch
        )

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.close_time = time.monotonic()
        self.opened = False

    @feedback
    def get_full_closed(self) -> bool:
        return (time.monotonic() - self.close_time) <= 0.5

    def execute(self) -> None:
        if self.opened:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.solenoid.set(DoubleSolenoid.Value.kForward)

    @feedback
    def game_piece_in_reach(self) -> bool:
        return not self.game_piece_switch.get()
