from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType

import ids


class Gripper:
    def __init__(self) -> None:
        self.opened = False

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

    def execute(self) -> None:
        if self.opened:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        else:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

    def game_piece_in_reach(self) -> bool:
        return self.game_piece_switch.get()
