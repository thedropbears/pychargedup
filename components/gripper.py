from wpilib import DoubleSolenoid, DigitalInput, PneumaticsModuleType

import ids


class Gripper:
    def __init__(self):
        self.opened = False

        self.solenoid = DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            ids.PcmChannels.Gripper.gripper_solenoid_forward,
            ids.PcmChannels.Gripper.gripper_solenoid_reverse,
        )

        self.game_piece_switch = DigitalInput(
            ids.DioChannels.Gripper.gripper_game_piece_switch
        )

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def execute(self):
        if self.opened:
            self.solenoid.set(value=DoubleSolenoid.Value.kForward)
        else:
            self.solenoid.set(value=DoubleSolenoid.Value.kReverse)

    def game_piece_in_reach(self):
        return self.game_piece_switch.get()
