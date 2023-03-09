import math
import wpilib
import time
from enum import Enum, auto
import random
from ids import PwmChannels

MAX_BRIGHTNESS = 75  # Between 0-255 of Value on HSV scale

Hsv = tuple[int, int, int]


class LedColors(Enum):
    # Use HSV to get nicer fading, hues are 0-180 so half usual hue
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    MAGENTA = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)

    DIM_BLUE = (120, 255, 40)
    DIM_RED = (0, 255, 40)


class DisplayType(Enum):
    RAINBOW = auto()
    SOLID = auto()
    PULSE = auto()
    FLASH = auto()
    MORSE = auto()


# creates a list of LEDData's from a List of (hsv col, repetitions)
def make_pattern(
    data: list[tuple[LedColors, int]]
) -> list[wpilib.AddressableLED.LEDData]:
    pattern_data = []
    for color, number in data:
        x = wpilib.AddressableLED.LEDData()
        x.setHSV(*color.value)
        pattern_data += [x] * number
    return pattern_data


class StatusLights:
    FLASH_FREQUENCY = 5
    PULSE_PERIOD = 1
    RAINBOW_PERIOD = 15

    def __init__(self) -> None:
        self.leds = wpilib.AddressableLED(PwmChannels.leds)
        strip_length = 144
        self.led_length = strip_length * 2
        self.leds.setLength(self.led_length)

        self.left_led_data = wpilib.AddressableLED.LEDData()
        self.right_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.left_led_data] * strip_length
        self.leds_data += [self.right_led_data] * strip_length

        self.start_time = time.monotonic()

        self.left_color: Hsv = LedColors.CYAN.value
        self.right_color: Hsv = LedColors.CYAN.value

        self.pattern = DisplayType.SOLID

        self._morse_message = ""
        self.pattern_start_time = time.monotonic()
        self.last_triggered = time.monotonic()

        self.choose_morse_message()
        self.left_led_data.setHSV(*self.left_color)
        self.right_led_data.setHSV(*self.right_color)
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set_color(self, color: LedColors) -> None:
        self.set_colors(color, color)

    def set_colors(self, left: LedColors, right: LedColors) -> None:
        self.left_color = left.value
        self.right_color = right.value

    def set_display_pattern(self, pattern: DisplayType):
        self.pattern = pattern

    def want_cone_left(self) -> None:
        self.set_colors(LedColors.YELLOW, LedColors.OFF)
        self.set_display_pattern(DisplayType.FLASH)

    def want_cone_right(self) -> None:
        self.set_colors(LedColors.OFF, LedColors.YELLOW)
        self.set_display_pattern(DisplayType.FLASH)

    def want_cube(self) -> None:
        """A side-ambiguous request for a cube"""
        self.set_color(LedColors.MAGENTA)
        self.set_display_pattern(DisplayType.FLASH)

    def off(self) -> None:
        self.set_color(LedColors.OFF)
        self.set_display_pattern(DisplayType.SOLID)

    def cone_onboard(self) -> None:
        self.set_color(LedColors.YELLOW)
        self.set_display_pattern(DisplayType.SOLID)

    def cube_onboard(self) -> None:
        self.set_color(LedColors.MAGENTA)
        self.set_display_pattern(DisplayType.SOLID)

    def calc_flash(self) -> tuple[Hsv, Hsv]:
        """Blinking (alternate off and on over time)."""
        elapsed_time = time.monotonic() - self.start_time
        is_lit = math.cos(self.FLASH_FREQUENCY * elapsed_time * math.tau) >= 0
        return self.calc_with_brightness(is_lit)

    def calc_with_brightness(self, brightness: float) -> tuple[Hsv, Hsv]:
        """Calculate the colors with the given relative brightness in [0,1]."""
        left = (
            self.left_color[0],
            self.left_color[1],
            round(self.left_color[2] * brightness),
        )
        right = (
            self.right_color[0],
            self.right_color[1],
            round(self.right_color[2] * brightness),
        )
        return left, right

    def calc_rainbow(self) -> Hsv:
        elapsed_time = time.monotonic() - self.start_time
        loop_time = self.RAINBOW_PERIOD / 3
        hue = round(180 * (elapsed_time / loop_time % 1))
        return (hue, 255, MAX_BRIGHTNESS)

    def calc_pulse(self) -> tuple[Hsv, Hsv]:
        """Fade in and out over time."""
        elapsed_time = time.monotonic() - self.start_time
        brightness = math.cos(elapsed_time * math.pi) / 2 + 0.5
        return self.calc_with_brightness(brightness)

    @property
    def _morse_length(self) -> int:
        # A dash is three times as long as a dot
        # A space between characters is three dots
        # A space between dots and dashes is one dot
        # A space between words is 7 dots
        return (
            self._morse_message.count(".")
            + 3 * self._morse_message.count("-")
            + 3 * self._morse_message.count(" ")
        )

    def calc_morse(self) -> Hsv:
        # Work out how far through the message we are
        DOT_LENGTH = 0.15  # seconds
        total_time = self._morse_length * DOT_LENGTH
        elapsed_time = time.monotonic() - self.pattern_start_time
        if elapsed_time > total_time:
            self.set_display_pattern(DisplayType.RAINBOW)
        running_total = 0.0
        for token in self._morse_message:
            if token == ".":
                running_total += 1.0 * DOT_LENGTH
            if token == "-" or token == " ":
                running_total += 3.0 * DOT_LENGTH
            if running_total > elapsed_time:
                # This is the current character
                if token == " ":
                    return LedColors.OFF.value
                else:
                    return LedColors.BLUE.value
        # Default - should never be hit
        return LedColors.OFF.value

    def choose_morse_message(self, _message=None) -> None:
        # Choose a morse message at random, unless specific message requested
        # Only use lowercase and spaces
        MESSAGES = [
            "KILL ALL HUMANS",
            "MORSE CODE IS FOR NERDS",
            "HONEYBADGER DONT CARE",
            "GLHF",
            "I HATE MORSE CODE",
        ]
        message = random.choice(MESSAGES) if _message is None else _message.upper()
        # Convert to dots and dashes
        MORSE_CODE_DICT = {
            "A": ".-",
            "B": "-...",
            "C": "-.-.",
            "D": "-..",
            "E": ".",
            "F": "..-.",
            "G": "--.",
            "H": "....",
            "I": "..",
            "J": ".---",
            "K": "-.-",
            "L": ".-..",
            "M": "--",
            "N": "-.",
            "O": "---",
            "P": ".--.",
            "Q": "--.-",
            "R": ".-.",
            "S": "...",
            "T": "-",
            "U": "..-",
            "V": "...-",
            "W": ".--",
            "X": "-..-",
            "Y": "-.--",
            "Z": "--..",
            "1": ".----",
            "2": "..---",
            "3": "...--",
            "4": "....-",
            "5": ".....",
            "6": "-....",
            "7": "--...",
            "8": "---..",
            "9": "----.",
            "0": "-----",
        }
        self._morse_message = ""
        for character in message:
            if character != " ":
                self._morse_message += " ".join(MORSE_CODE_DICT[character]) + "  "
            else:
                self._morse_message += "     "
        # Add more space at the end
        self._morse_message += "  "

    def execute(self) -> None:
        if self.pattern == DisplayType.FLASH:
            left, right = self.calc_flash()
        elif self.pattern == DisplayType.PULSE:
            left, right = self.calc_pulse()
        # These are the same color on all the LEDs
        elif self.pattern == DisplayType.RAINBOW:
            left = right = self.calc_rainbow()
        elif self.pattern == DisplayType.MORSE:
            left = right = self.calc_morse()
        else:
            # Fallback to solid
            left, right = self.left_color, self.right_color

        self.left_led_data.setHSV(*left)
        self.right_led_data.setHSV(*right)
        self.leds.setData(self.leds_data)
