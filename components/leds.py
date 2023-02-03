import math
import wpilib
import time
from enum import Enum, auto
from typing import List, Tuple
from utilities.scalers import scale_value


MAX_BRIGHTNESS = 100  # Between 0-255 of Value on HSV scale


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
    ALTERNATING = auto()
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

    def colour(self) -> LedColours:
        if self._value_ == 1:
            return LedColours.GREEN
        elif self._value_ == 2:
            return LedColours.RED
        return LedColours.PINK


# creates a list of LEDData's from a List of (hsv col, repetitions)
def make_pattern(
    data: list[tuple[LedColours, int]]
) -> list[wpilib.AddressableLED.LEDData]:
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
    ALTERNATING_PERIOD = 60

    def __init__(self):
        self.led_length = 60

        self.start_time = time.monotonic()

        self.colour = (0, 0, 0)

        self.pattern = DisplayType.SOLID

        self.side = PickupFromSide.RIGHT

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set_colour(self, colour: LedColours):
        self.colour = colour.value

    def set_piece(self, piece: Piece):
        if piece == Piece.CONE:
            self.set_colour(LedColours.YELLOW)
        elif piece == Piece.CUBE:
            self.set_colour(LedColours.VIOLET)
        elif piece == Piece.NONE:
            self.set_colour(LedColours.OFF)

    def set_state(self, state: RobotState):
        if state == RobotState.PICKED_UP_PIECE:
            self.pattern = DisplayType.SOLID
        elif state == RobotState.LOOKING_FOR_PIECE:
            self.pattern = DisplayType.HALF_HALF
        elif RobotState.OTHER:
            self.set_colour(LedColours.OFF)

    def set_intake_side(self, side: PickupFromSide):
        self.side = side

    def set_display_pattern(self, pattern: DisplayType):
        self.pattern = pattern

    def set(
        self,
        piece: Piece,
        state: RobotState,
        side: PickupFromSide,
    ):
        self.set_piece(piece)
        self.set_state(state)
        self.set_intake_side(side)

    def calc_half(self):
        led_data = []
        for i in range(self.led_length):
            if i < round(self.led_length / 2):
                led_data.append(wpilib.AddressableLED.LEDData(*self.colour))
            else:
                led_data.append(
                    wpilib.AddressableLED.LEDData(*self.side.colour().value)
                )
        self.leds.setData(led_data[: self.led_length])

    def calc_solid(self) -> tuple[int, int, int]:
        return self.colour

    def calc_flash(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        brightness = math.cos(self.FLASH_PERIOD * elapsed_time / math.pi) / 2 + 1
        return (self.colour[0], self.colour[1], self.colour[2] * round(brightness))

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
        return (self.colour[0], self.colour[1], round(self.colour[2] * brightness))

    def calc_alternating(self):
        elapsed_time = time.monotonic() - self.start_time
        # flash, but only every second led
        # then, the other leds flash
        # and so on?

        # make patter is redundant here but its easier to just keep it here
        pattern = make_pattern([(LedColours.BLUE, 1), (LedColours.OFF, 1)])
        position = round(
            scale_value(
                elapsed_time % self.ALTERNATING_PERIOD,
                0,
                self.ALTERNATING_PERIOD,
                self.led_length,
                -len(pattern),
            )
        )
        led_data = []
        if position > 0:
            led_data.extend(
                [
                    wpilib.AddressableLED.LEDData(
                        LedColours.BLUE.value[0],
                        LedColours.BLUE.value[1],
                        LedColours.BLUE.value[2],
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
                        LedColours.BLUE.value[0],
                        LedColours.BLUE.value[1],
                        LedColours.BLUE.value[2],
                    ),
                ]
                * (leds_left // 2)
            )
        self.leds.setData(led_data[: self.led_length])

    def execute(self):
        colour = self.calc_solid()
        if self.pattern == DisplayType.SOLID:
            colour = self.calc_solid()
        elif self.pattern == DisplayType.FLASH:
            colour = self.calc_flash()
        elif self.pattern == DisplayType.PULSE:
            colour = self.calc_pulse()
        elif self.pattern == DisplayType.RAINBOW:
            colour = self.calc_rainb()
        elif self.pattern == DisplayType.HALF_HALF:
            self.calc_half()
            return
        elif self.pattern == DisplayType.PACMAN:
            self.calc_pacma()
            return  # pacman sets LEDs
        elif self.pattern == DisplayType.ALTERNATING:
            self.calc_alternating()
            return  # alternating sets LEDs

        self.single_led_data.setHSV(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
