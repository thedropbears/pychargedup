import math
import wpilib
import time
from enum import Enum, auto
from utilities.scalers import scale_value
import random


MAX_BRIGHTNESS = 100  # Between 0-255 of Value on HSV scale


class LedColors(Enum):
    # Use HSV to get nicer fading, hues are 0-180 so half usual hue
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    PINK = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    VIOLET = (180, 45, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)


class DisplayType(Enum):
    PACMAN = auto()
    RAINBOW = auto()
    SOLID = auto()
    PULSE = auto()
    FLASH = auto()
    ALTERNATING = auto()
    HALF_HALF = auto()
    MORSE = auto()
    WOLFRAM_AUTOMATA = auto()


class RobotState(Enum):
    PICKED_UP_PIECE = auto()
    LOOKING_FOR_PIECE = auto()
    OTHER = auto()


class PieceColour(Enum):
    CONE = LedColors.YELLOW
    CUBE = LedColors.VIOLET
    NONE = LedColors.OFF


class PickupFromSide(Enum):
    LEFT = LedColors.GREEN
    RIGHT = LedColors.RED
    NONE = LedColors.OFF


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


# Runs Wolfram cellular automata, randomly transitioning between rules
class WolframAutomata:
    def __init__(self, n: int) -> None:
        self.rule = 4
        self.n = n
        self.m = n // 2

        self.world = [False] * n
        self.world[self.m] = True
        self.old_world = self.world[:]
        self.ages = [0] * n

        self.leds_data = [wpilib.AddressableLED.LEDData(0, 0, 0) for _ in range(n)]

        self.gen = 0
        self.gen_rule = 0
        self.genspan_min = 8
        self.genspan_max = 32
        self.genspan = self.genspan_min  # How much will the current rule last

        self.length_thresh = (
            8  # Prevent changes of at least this many cells in a row over 1 generation
        )
        self.age_hue_mul = 0.5

    def reset_rule(self) -> None:
        self.gen_rule = 0
        self.genspan = random.randint(self.genspan_min, self.genspan_max)
        self.rule = random.randint(0, 255)

    def step(self) -> None:
        new_world = [0] * self.n
        for i in range(self.n):
            r = (
                (self.world[(i + 1) % self.n] << 0)
                + (self.world[i] << 1)
                + (self.world[(i - 1) % self.n] << 2)
            )
            new_world[i] = (self.rule >> r) & 1 != 0
        if self.gen_rule > self.genspan:
            if (
                new_world == self.world
                or new_world == self.old_world
                or all(
                    self.world[i] == new_world[(i - 1) % self.n] for i in range(self.n)
                )
                or all(
                    self.world[i] == new_world[(i + 1) % self.n] for i in range(self.n)
                )
            ):
                self.reset_rule()  # Reset rule if nothing interesting happened
            if random.random() < (self.gen_rule - self.genspan) * 0.001:
                self.reset_rule()
        change = [a ^ b for a, b in zip(self.world, new_world)]
        for i in range(self.n):
            if all(change[(i + j) % self.n] for j in range(self.length_thresh)):
                new_world = self.old_world[:]
                self.world = self.old_world[:]
                self.reset_rule()
                break

        if all(new_world) or not any(new_world):
            new_world = self.old_world[:]
            self.reset_rule()
        self.old_world = self.world[:]
        self.world = new_world[:]
        for i in range(self.n):
            if self.world[i]:
                self.ages[i] += 1
            else:
                self.ages[i] = 0
        self.gen += 1
        self.gen_rule += 1

    def age_to_hsv(self, a: int) -> tuple[int, int, int]:
        return (
            (self.gen + int(a * self.genspan * self.age_hue_mul)) % 180,
            255,
            MAX_BRIGHTNESS,
        )

    def set_leds_data(self) -> None:
        for i in range(self.n):
            if self.world[i]:
                self.leds_data[i].setHSV(*self.age_to_hsv(self.ages[i]))
            else:
                self.leds_data[i].setRGB(0, 0, 0)


class StatusLights:
    leds: wpilib.AddressableLED

    FLASH_PERIOD = 0.4
    PULSE_PERIOD = 1
    PACMAN_PERIOD = 60
    RAINBOW_PERIOD = 15
    ALTERNATING_PERIOD = 60
    WOLFRAM_PERIOD = 0.1

    def __init__(self):
        self.led_length = 131

        self.start_time = time.monotonic()

        self.color = (0, 0, 0)

        self.pattern = DisplayType.SOLID

        self.side = PickupFromSide.RIGHT

        self._morse_message = ""
        self.pattern_start_time = time.monotonic()
        self.last_triggered = time.monotonic()

        self.wolfram = WolframAutomata(self.led_length)

    def setup(self) -> None:
        self.choose_morse_message()
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set_color(self, color: LedColors):
        self.color = color.value

    def set_piece(self, piece: PieceColour):
        self.set_color(piece.value)

    def set_state(self, state: RobotState):
        if state == RobotState.PICKED_UP_PIECE:
            self.pattern = DisplayType.SOLID
        elif state == RobotState.LOOKING_FOR_PIECE:
            self.pattern = DisplayType.HALF_HALF
        elif RobotState.OTHER:
            self.set_color(LedColors.OFF)

    def set_intake_side(self, side: PickupFromSide):
        self.side = side

    def set_display_pattern(self, pattern: DisplayType):
        self.pattern = pattern

    def set(
        self,
        piece: PieceColour,
        state: RobotState,
        side: PickupFromSide,
    ):
        self.set_piece(piece)
        self.set_state(state)
        self.set_intake_side(side)

    def want_cone_left(self) -> None:
        self.set(PieceColour.CONE, RobotState.LOOKING_FOR_PIECE, PickupFromSide.LEFT)

    def want_cube_left(self) -> None:
        self.set(PieceColour.CUBE, RobotState.LOOKING_FOR_PIECE, PickupFromSide.LEFT)

    def want_cone_right(self) -> None:
        self.set(PieceColour.CONE, RobotState.LOOKING_FOR_PIECE, PickupFromSide.RIGHT)

    def want_cube_right(self) -> None:
        self.set(PieceColour.CUBE, RobotState.LOOKING_FOR_PIECE, PickupFromSide.RIGHT)

    def cone_onboard(self) -> None:
        self.set(PieceColour.CONE, RobotState.PICKED_UP_PIECE, PickupFromSide.NONE)

    def cube_onboard(self) -> None:
        self.set(PieceColour.CONE, RobotState.PICKED_UP_PIECE, PickupFromSide.NONE)

    def calc_half(self) -> None:
        led_data: list[wpilib.AddressableLED.LEDData] = []
        for i in range(self.led_length):
            if i < round(self.led_length / 2):
                led_data.append(wpilib.AddressableLED.LEDData(0, 0, 0))
                led_data[-1].setHSV(*self.color)
            else:
                led_data.append(wpilib.AddressableLED.LEDData(0, 0, 0))
                led_data[-1].setHSV(*LedColors(self.side.value).value)
        self.leds.setData(led_data[: self.led_length])

    def calc_solid(self) -> tuple[int, int, int]:
        return self.color

    def calc_flash(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        brightness = math.cos(self.FLASH_PERIOD * elapsed_time / math.pi) / 2 + 1
        return (self.color[0], self.color[1], self.color[2] * round(brightness))

    def calc_rainbow(self) -> tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.start_time
        loop_time = self.RAINBOW_PERIOD / 3
        hue = round(180 * (elapsed_time / loop_time % 1))
        return (hue, 255, MAX_BRIGHTNESS)

    def calc_pacman(self):
        elapsed_time = time.monotonic() - self.start_time
        # find the pattern of leds and its position
        if elapsed_time % self.PACMAN_PERIOD < self.PACMAN_PERIOD / 2:
            pattern = make_pattern(
                [
                    (LedColors.ORANGE, 2),
                    (LedColors.OFF, 2),
                    (LedColors.PINK, 2),
                    (LedColors.OFF, 2),
                    (LedColors.CYAN, 2),
                    (LedColors.OFF, 2),
                    (LedColors.RED, 2),
                    (LedColors.OFF, 5),
                    (LedColors.YELLOW, 3),
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
                    (LedColors.BLUE, 2),
                    (LedColors.OFF, 2),
                    (LedColors.BLUE, 2),
                    (LedColors.OFF, 2),
                    (LedColors.BLUE, 2),
                    (LedColors.OFF, 2),
                    (LedColors.BLUE, 2),
                    (LedColors.OFF, 5),
                    (LedColors.YELLOW, 3),
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

    def calc_alternating(self):
        elapsed_time = time.monotonic() - self.start_time
        # flash, but only every second led
        # then, the other leds flash
        # and so on?

        # make patter is redundant here but its easier to just keep it here
        pattern = make_pattern([(LedColors.BLUE, 1), (LedColors.OFF, 1)])
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
                        LedColors.BLUE.value[0],
                        LedColors.BLUE.value[1],
                        LedColors.BLUE.value[2],
                    ),
                    wpilib.AddressableLED.LEDData(0, 0, 0),
                ]
                * (position // 2)
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
                        LedColors.BLUE.value[0],
                        LedColors.BLUE.value[1],
                        LedColors.BLUE.value[2],
                    ),
                ]
                * (leds_left // 2)
            )
        self.leds.setData(led_data[: self.led_length])

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

    def calc_morse(self) -> tuple[int, int, int]:
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

    def calc_wolfram(self):
        now = time.monotonic()
        if now - self.last_triggered > self.WOLFRAM_PERIOD:
            self.last_triggered = now
            self.wolfram.step()
            self.wolfram.set_leds_data()
            self.leds.setData(self.wolfram.leds_data)

    def execute(self):
        # use the current color as a fallback if (for whatever reason) there is no pattern set.
        color = self.color
        if self.pattern == DisplayType.SOLID:
            color = self.calc_solid()
        elif self.pattern == DisplayType.FLASH:
            color = self.calc_flash()
        elif self.pattern == DisplayType.PULSE:
            color = self.calc_pulse()
        elif self.pattern == DisplayType.RAINBOW:
            color = self.calc_rainbow()
        elif self.pattern == DisplayType.MORSE:
            color = self.calc_morse()
        # Those set data directly
        elif self.pattern == DisplayType.HALF_HALF:
            self.calc_half()
            return
        elif self.pattern == DisplayType.PACMAN:
            self.calc_pacman()
            return
        elif self.pattern == DisplayType.ALTERNATING:
            self.calc_alternating()
            return
        elif self.pattern == DisplayType.WOLFRAM_AUTOMATA:
            self.calc_wolfram()
            return

        self.single_led_data.setHSV(color[0], color[1], color[2])
        self.leds.setData(self.leds_data)
