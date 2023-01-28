import wpilib


class Gripper:
    gripper_solenoid: wpilib.DoubleSolenoid

    def __init__(self):
        self.is_gripper_open = True

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def execute(self):
        if self.is_gripper_open:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kReverse)
