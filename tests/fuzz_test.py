from __future__ import annotations

import random
import typing

import hal
import wpilib.simulation

if typing.TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController


def rand_bool() -> bool:
    return random.getrandbits(1) != 0


def rand_axis() -> float:
    """Get a random number between -1 and 1."""
    return random.random() * 2 - 1


def rand_pov() -> int:
    """Pick a random POV hat value."""
    return random.choice((-1, 0, 45, 90, 135, 180, 225, 270, 315))


class AllTheThings:
    """Fuzzer for robot hardware inputs."""

    def __init__(self) -> None:
        self.dios = [
            dio
            for dio in map(wpilib.simulation.DIOSim, range(hal.getNumDigitalChannels()))
            if dio.getInitialized()
        ]

    def fuzz(self) -> None:
        for dio in self.dios:
            if dio.getIsInput():  # pragma: no branch
                dio.setValue(rand_bool())


class DSInputs:
    """Fuzzer for HIDs attached to the driver station."""

    def __init__(self) -> None:
        self.gamepad = wpilib.simulation.XboxControllerSim(0)
        self.joystick = wpilib.simulation.JoystickSim(1)

    def fuzz(self) -> None:
        fuzz_xbox_gamepad(self.gamepad)
        fuzz_joystick(self.joystick)


def fuzz_joystick(joystick: wpilib.simulation.JoystickSim) -> None:
    """Fuzz a Logitech Extreme 3D Pro flight stick."""
    for axis in range(5):
        joystick.setRawAxis(axis, rand_axis())
    for button in range(12):
        joystick.setRawButton(button, rand_bool())
    joystick.setPOV(rand_pov())


def fuzz_xbox_gamepad(gamepad: wpilib.simulation.XboxControllerSim) -> None:
    """Fuzz an XInput gamepad."""
    gamepad.setLeftX(rand_axis())
    gamepad.setLeftY(rand_axis())
    gamepad.setRightX(rand_axis())
    gamepad.setRightY(rand_axis())
    gamepad.setLeftTriggerAxis(random.random())
    gamepad.setRightTriggerAxis(random.random())
    for button in range(10):
        gamepad.setRawButton(button, rand_bool())
    gamepad.setPOV(rand_pov())


def _test_fuzz(control: TestController, fuzz_disabled_hids: bool) -> None:
    with control.run_robot():
        things = AllTheThings()
        hids = DSInputs()

        # Disabled mode
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        things.fuzz()
        if fuzz_disabled_hids:
            hids.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)

        # Autonomous mode
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=True)

        # Transition between autonomous and teleop
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)

        # Teleop
        for _ in range(10):
            things.fuzz()
            hids.fuzz()
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)


def test_fuzz(control: TestController) -> None:
    _test_fuzz(control, fuzz_disabled_hids=False)


def test_fuzz_disabled(control: TestController) -> None:
    _test_fuzz(control, fuzz_disabled_hids=True)
