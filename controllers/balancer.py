import math
from magicbot import StateMachine, state

from components.chassis import Chassis
from utilities.game import get_team


class ChargeStation(StateMachine):
    chassis: Chassis

    DRIVE_ON_SPEED = 0.5
    BALANCE_SPEED = 0.3
    LEVEL_THRESHOLD = math.radians(7)

    BLUE_CHARGE_STATION_X = 3.15165
    RED_CHARGE_STATION_X = 12.70255

    def __init__(self) -> None:
        self.drive_direction_positive = True

    def get_direction(self):
        team = get_team().name
        pose = self.chassis.get_pose()
        if team == "kBlue":
            self.drive_direction_positive = pose.X() < self.BLUE_CHARGE_STATION_X
        else:
            self.drive_direction_positive = pose.X() < self.RED_CHARGE_STATION_X

    @state(first=True)
    def drive_on(self, initial_call) -> None:
        """Drive until we detect we are on the station"""
        if initial_call:
            self.get_direction()
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

    def start(self) -> None:
        self.engage()
