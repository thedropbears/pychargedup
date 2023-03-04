import math
from magicbot import StateMachine, state
from utilities.game import get_team
from wpimath.filter import Debouncer

from components.chassis import Chassis


class ChargeStation(StateMachine):
    chassis: Chassis

    DRIVE_ON_SPEED = 0.5
    BALANCE_SPEED = 0.2
    LEVEL_THRESHOLD = math.radians(4)
    TILT_RATE_THRESHOLD = math.radians(10)
    DEBOUNCE_TIME = 0.5

    BLUE_CHARGE_STATION_X = 3.15165
    RED_CHARGE_STATION_X = 12.70255

    def __init__(self) -> None:
        self.drive_direction_positive = True
        self.debouncer = Debouncer(self.DEBOUNCE_TIME, Debouncer.DebounceType.kRising)

    def get_direction(self):
        team = get_team().name
        pose = self.chassis.get_pose()
        if team == "kBlue":
            self.drive_direction_positive = pose.X() < self.BLUE_CHARGE_STATION_X
        else:
            self.drive_direction_positive = pose.X() < self.RED_CHARGE_STATION_X

    @state(first=True)
    def balance(self) -> None:
        """Drive until the station tilts"""
        tilt = self.chassis.get_tilt()
        rate = self.chassis.get_tilt_rate()

        self.chassis.drive_local(
            self.BALANCE_SPEED if tilt < 0 else -self.BALANCE_SPEED,
            0,
            0,
        )

        if self.debouncer.calculate(abs(rate) >= self.TILT_RATE_THRESHOLD):
            self.done()

    def start(self) -> None:
        self.engage()
