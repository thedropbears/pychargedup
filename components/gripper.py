import wpilib


class Gripper:
    gripper_solenoid: wpilib.DoubleSolenoid
    gripper_game_piece_switch: wpilib.DigitalInput

    def __init__(self):
        self.opened = False

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def execute(self):
        if self.opened:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kReverse)

    def game_piece_in_reach(self):
        return self.gripper_game_piece_switch.get()
