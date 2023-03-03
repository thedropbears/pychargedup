import math
from magicbot import StateMachine, state

from controllers.movement import Movement


class ChargeStation(StateMachine):
    movement: Movement

    DRIVE_ON_SPEED = 0.5
    BALANCE_SPEED = 0.3
    LEVEL_THRESHOLD = math.radians(7)

    def __init__(self) -> None:
        self.drive_direction_positive = True

    @state
    def drive_on(self) -> None:
        """Drive until we detect we are on the station"""
        ...

    @state
    def balance(self) -> None:
        """Drive until the station tilts"""
        ...

    def start(self, positive_x: bool) -> None:
        self.engage()
        self.drive_direction_positive = positive_x
