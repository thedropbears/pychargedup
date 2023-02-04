from components.leds import (
    StatusLights,
    LedColours,
    DisplayType,
)


class LedController:
    status_lights: StatusLights

    def __init__(self):
        self.enabled = False

    def on_enabled(self):
        self.enabled = True

    def on_disabled(self):
        self.enabled = False

    def execute(self) -> None:
        self.status_lights.set_colour(LedColours.RED)
        self.status_lights.set_display_pattern(DisplayType.PULSE)
