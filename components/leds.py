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
    OFF = (0, 0, 0)


class DisplayType(Enum):
    PACMAN = auto()
    RAINBOW = auto()
    SOLID = auto()
    PULSE = auto()
    FLASH = auto()

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

    def __init__(self):
        self.led_length = 1

        self.start_time = time.monotonic()

        self.color = (0, 0, 0)

        self.pattern = DisplayType.SOLID

        self._message = ""


    def setup(self) -> None:
        self.choose_morse_message()
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def calc_solid(self) -> tuple[int, int, int]:
        return self.color

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
        return (self.color[0], self.color[1], round(self.color[2] * brightness))

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
            case DisplayType.PACMAN:
                self.calc_pacma()
                return

        self.single_led_data.setHSV(colors[0], colors[1], colors[2])
        self.leds.setData(self.leds_data)

