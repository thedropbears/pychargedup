import wpilib


class Claw:
    def __init__(self):
        self.gripper_solenoid = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, forwardChannel=0, reverseChannel=1
        )
        self.is_gripper_open = True

    def open_gripper(self):
        is_gripper_open = True

    def close_gripper(self):
        is_gripper_open = False

    def engage(self):
        if self.is_gripper_open:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.gripper_solenoid.set(value=wpilib.DoubleSolenoid.Value.kReverse)
