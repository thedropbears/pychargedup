import wpilib


class Gripper:
    solenoid: wpilib.DoubleSolenoid
    game_piece_switch: wpilib.DigitalInput

    def __init__(self):
        self.opened = False

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def execute(self):
        if self.opened:
            self.solenoid.set(value=wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.solenoid.set(value=wpilib.DoubleSolenoid.Value.kReverse)

    def game_piece_in_reach(self):
        return self.game_piece_switch.get()
