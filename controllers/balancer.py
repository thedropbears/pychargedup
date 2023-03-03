import math
from magicbot import StateMachine, state

from components.chassis import Chassis


class ChargeStation(StateMachine):
    chassis: Chassis

    DRIVE_ON_SPEED = 0.5
    BALANCE_SPEED = 0.3
    LEVEL_THRESHOLD = math.radians(7)

    def __init__(self) -> None:
        self.drive_direction_positive = True

    @state(first=True)
    def drive_on(self) -> None:
        """Drive until we detect we are on the station"""
        self.chassis.drive_field(
            self.DRIVE_ON_SPEED
            if self.drive_direction_positive
            else -self.DRIVE_ON_SPEED,
            0,
            0,
        )
        if abs(self.chassis.get_tilt()) > self.LEVEL_THRESHOLD:
            self.next_state("balance")

    @state
    def balance(self) -> None:
        """Drive until the station tilts"""
        self.chassis.drive_field(
            self.BALANCE_SPEED
            if self.drive_direction_positive
            else -self.BALANCE_SPEED,
            0,
            0,
        )
        if abs(self.chassis.get_tilt()) < self.LEVEL_THRESHOLD:
            self.done()

    def start(self, positive_x: bool) -> None:
        self.engage()
        self.drive_direction_positive = positive_x
