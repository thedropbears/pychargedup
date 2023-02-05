from components.leds import StatusLights, LedColors, DisplayType
from components.gripper import Gripper
from components.intake import Intake


class LedController:
    status_lights: StatusLights
    gripper: Gripper
    intake: Intake

    def __init__(self) -> None:
        ...

    def execute(self) -> None:
        CONE = self.gripper.get_full_closed() and CHECK_THAT_PIECE_ISNT_A_CUBE
        CUBE = self.intake.is_game_piece_present()
        LEFT = LEFT_CHECK
        RIGHT = RIGHT_CHECK
        WANTS_CONE = WANTS_CONE_CHECK
        WANTS_CUBE = WANTS_CUBE_CHECK
        if WANTS_CONE:
            if LEFT:
                self.status_lights.want_cone_left()
            elif RIGHT:
                self.status_lights.want_cone_right()
        elif WANTS_CUBE:
            if LEFT:
                self.status_lights.want_cube_left()
            elif RIGHT:
                self.status_lights.want_cube_right()
        elif CONE:
            self.status_lights.cone_onboard()
        elif CUBE:
            self.status_lights.cube_onboard()
        else:
            self.status_lights.set_display_pattern(DisplayType.SOLID)
            self.status_lights.set_color(LedColors.OFF)
