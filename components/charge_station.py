from enum import Enum, auto
from controllers.movement import Movement
from wpimath.geometry import Pose2d
from components.chassis import Chassis
import time

"""
Allows for {slippage}% error for value to be close to

slippage goes both ways, so setting {slippage} to 1 would allow 1% error over {wanted} and 1% under {wanted}
"""


def slippage(value: float, slippage: float, wanted: float) -> bool:
    floor = value * (1 - (slippage / 100))
    high = value * (1 + (slippage / 100))
    return floor < wanted < high


class ChargeStationState(Enum):
    NOT_ALIGNED = auto()
    OFF         = auto()
    MIDWAY      = auto()
    ON          = auto()


class ChargeStation:
    chassis: Chassis

    TILTED_STATION_DEGREES = 11

    ADJUSTMENT = 1
    """ The velocity to move the chassis at to adjust tha position """

    REQUIRED_SECONDS = 5.
    """ seconds at 0deg to count as level"""
    steps = 0.
    start_time = 0.

    def __init__(self):
        self.state = ChargeStationState.OFF
        
    def get_angle(self) -> float:
        return self.chassis.imu.getAngle()

    def set_state(self, state: ChargeStationState) -> None:
        self.state = state

    def move_off(self) -> None:
        # check if we are now on the charge station
        if slippage(
            self.get_angle(),
            2,  # 2% slippage, so if we are 2% of 11deg it still counts as being on the dashboard
            self.TILTED_STATION_DEGREES,
        ):
            self.state = ChargeStationState.MIDWAY
        else:
            self.chassis.drive_local(0, self.ADJUSTMENT, 0)

    def move_midway(self) -> None:
        # check if the charge station is level
        if slippage(
            self.get_angle(),
            2,  # 2% slippage, so if we are 2% of 0deg it still counts as being on the dashboard
            0,
        ):
            if self.steps == 0:
                self.start_time = time.monotonic()
            elif self.steps >= self.start_time + self.REQUIRED_SECONDS:
                self.state = ChargeStationState.ON
            else:
                self.steps = time.monotonic()
        elif self.get_angle() < 0:
            self.steps = 0  # set the steps at 0deg to 0
            self.chassis.drive_local(0, -self.ADJUSTMENT, 0)
        else:
            self.steps = 0  # ''
            self.chassis.drive_local(0, self.ADJUSTMENT, 0)

    def start(self) -> None:
        """
        a function to tell the controller to start moving onto the station
        """
        self.state = ChargeStationState.OFF

    def done(self) -> None:
        """
        a function to tell the controller to stop moving onto the station
        """
        self.state = ChargeStationState.NOT_ALIGNED

    def execute(self) -> None:
        if self.state == ChargeStationState.OFF:
            self.move_off()
        elif self.state == ChargeStationState.MIDWAY:
            self.move_midway()
        elif (
            self.state == ChargeStationState.ON
            or self.state == ChargeStationState.NOT_ALIGNED
        ):
            return
            """
            ON means that we dont need to move, NOT_ALIGNED means that we are not aligned with the charge station and should not move
            these are different values for readability
            """
