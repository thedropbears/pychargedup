import math
import wpilib
import random
import time
from enum import Enum, auto
from typing import List, Optional, Tuple
from utilities.scalers import scale_value


MAX_BRIGHTNESS = 180  # Between 0-255 of Value on HSV scale


class LedColours(Enum):
    # Use HSV to get nicer fading, hues are 0-180 so half usual hue
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    PINK = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    VIOLET = (300, 45, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)


class DisplayType(Enum):
    PACMAN = auto()
    RAINBOW = auto()
    SOLID = auto()
    PULSE = auto()
    FLASH = auto()
    IDK = auto()
    HALF_HALF = auto()


class RobotState(Enum):
    PICKED_UP_PIECE = auto()
    LOOKING_FOR_PIECE = auto()
    OTHER = auto()


class Piece(Enum):
    CONE = auto()
    CUBE = auto()
    NONE = auto()


class PickupFromSide(Enum):
    LEFT = auto()
    RIGHT = auto()


# creates a list of LEDData's from a List of (hsv col, repetitions)
def make_pattern(
    data: List[Tuple[LedColours, int]]
) -> List[wpilib.AddressableLED.LEDData]:
    pattern_data = []
    for colour, number in data:
        x = wpilib.AddressableLED.LEDData()
        x.setHSV(*colour.value)
        pattern_data += [x] * number
    return pattern_data


class StatusLights:
    leds: wpilib.AddressableLED

    FLASH_PERIOD = 0.4
    PULSE_PERIOD = 1
    PACMAN_PERIOD = 60
    RAINBOW_PERIOD = 15
    IDK_PERIOD = 60

    def __init__(self):
        self.led_length = 1

        self.start_time = time.monotonic()

        self.color = (0, 0, 0)

        self.pattern = DisplayType.SOLID

        self.side = PickupFromSide.RIGHT

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set_color(self, color: tuple[int, int, int] | LedColours):
        self.color = color

    def set_piece(self, piece: Piece):
        match piece:
            case Piece.CONE:
                self.set_color(LedColours.YELLOW)
            case Piece.CUBE:
                self.set_color(LedColours.VIOLET)
            case Piece.NONE:
                self.set_color(LedColours.OFF)

    def set_state(self, state: RobotState):
        match state:
            case RobotState.PICKED_UP_PIECE:
                self.pattern = DisplayType.SOLID
            case RobotState.LOOKING_FOR_PIECE:
                self.pattern = DisplayType.HALF_HALF
            case RobotState.OTHER:
                self.set_color(LedColours.OFF)

    def set_intake_side(self, side: PickupFromSide):
        self.side = side

    def set_display_pattern(self, pattern: DisplayType):
        self.pattern = pattern

    def set(
        self,
        color: tuple[int, int, int],
        piece: Piece,
        state: RobotState,
        side: PickupFromSide,
    ):
        # self.set_color(color)
        self.set_piece(piece)
        self.set_state(state)
        self.set_intake_side(side)

    def calc_half(self):
        led_data = []
        for i in range(self.led_length):
            if i < round(self.led_length / 2):
                led_data.append(self.color)
            else:
                led_data.append(
                    LedColours.RED
                    if self.side == PickupFromSide.LEFT
                    else LedColours.GREEN
                )
        self.leds.setData(led_data[: self.led_length])

    def calc_solid(self) -> tuple[int, int, int]:
        return self.color

    def calc_flash(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        brightness = math.cos(self.FLASH_PERIOD * elapsed_time / math.pi) / 2 + 1
        return (self.color[0], self.color[1], self.color[2] * round(brightness))

    def calc_rainb(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        loop_time = self.RAINBOW_PERIOD / 3
        hue = round(180 * (elapsed_time / loop_time % 1))
        return (hue, 255, MAX_BRIGHTNESS)

    def calc_pacma(self):
        elapsed_time = time.monotonic() - self.start_time
        # find the pattern of leds and its position
        if elapsed_time % self.PACMAN_PERIOD < self.PACMAN_PERIOD / 2:
            pattern = make_pattern(
                [
                    (LedColours.ORANGE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.PINK, 2),
                    (LedColours.OFF, 2),
                    (LedColours.CYAN, 2),
                    (LedColours.OFF, 2),
                    (LedColours.RED, 2),
                    (LedColours.OFF, 5),
                    (LedColours.YELLOW, 3),
                ]
            )
            pacman_position = scale_value(
                elapsed_time % self.PACMAN_PERIOD,
                0,
                self.PACMAN_PERIOD / 2,
                -len(pattern),
                self.led_length,
            )
        else:
            pattern = make_pattern(
                [
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 5),
                    (LedColours.YELLOW, 3),
                ]
            )
            pacman_position = scale_value(
                elapsed_time % self.PACMAN_PERIOD,
                self.PACMAN_PERIOD / 2,
                self.PACMAN_PERIOD,
                self.led_length,
                -len(pattern),
            )
        pacman_position = round(pacman_position)
        # create a list of LEDData's with the pattern surrounded by off led's
        led_data = []
        if pacman_position > 0:
            led_data.extend(
                [wpilib.AddressableLED.LEDData(0, 0, 0)] * math.floor(pacman_position)
            )
            led_data.extend(pattern)
        else:
            led_data.extend(pattern[-pacman_position:-1])
        leds_left = self.led_length - len(led_data)
        if leds_left > 0:
            led_data.extend([wpilib.AddressableLED.LEDData(0, 0, 0)] * leds_left)
        self.leds.setData(led_data[: self.led_length])

    def calc_pulse(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        brightness = math.cos(elapsed_time * math.pi) / 2 + 0.5
        return (self.color[0], self.color[1], round(self.color[2] * brightness))

    def calc_idk(self):
        elapsed_time = time.monotonic() - self.start_time
        # flash, but only every second led
        # then, the other leds flash
        # and so on?

        # make patter is redundant here but its easier to just keep it here
        pattern = make_pattern([(LedColours.BLUE, 1), (LedColours.OFF, 1)])
        position = round(
            scale_value(
                elapsed_time % self.IDK_PERIOD,
                0,
                self.IDK_PERIOD,
                self.led_length,
                -len(pattern),
            )
        )
        led_data = []
        if position > 0:
            led_data.extend(
                [
                    wpilib.AddressableLED.LEDData(
                        LedColours.BLUE[0], LedColours.BLUE[1], LedColours.BLUE[2]
                    ),
                    wpilib.AddressableLED.LEDData(0, 0, 0),
                ]
                * math.floor(position / 2)
            )
            led_data.extend(pattern)
        else:
            led_data.extend(pattern[-position:-1])
        leds_left = self.led_length - len(led_data)
        if leds_left > 0:
            led_data.extend(
                [
                    wpilib.AddressableLED.LEDData(0, 0, 0),
                    wpilib.AddressableLED.LEDData(
                        LedColours.BLUE[0], LedColours.BLUE[1], LedColours.BLUE[2]
                    ),
                ]
                * (leds_left / 2)
            )
        self.leds.setData(led_data[: self.led_length])

    def execute(self):
        match self.pattern:
            case DisplayType.SOLID:
                colors = self.calc_solid()
            case DisplayType.FLASH:
                colors = self.calc_flash()
            case DisplayType.PULSE:
                colors = self.calc_pulse()
            case DisplayType.RAINBOW:
                colors = self.calc_rainb()
            case DisplayType.HALF_HALF:
                self.calc_half()
                return
            case DisplayType.PACMAN:
                self.calc_pacma()
                return  # pacman sets LEDs
            case DisplayType.IDK:
                self.calc_idk()
                return  # whatever this is sets LEDs

        self.single_led_data.setHSV(colors[0], colors[1], colors[2])
        self.leds.setData(self.leds_data)
