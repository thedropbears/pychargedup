"""from components.leds import StatusLights, LedColors, DisplayType, PieceColour, PickupFromSide
from components.gripper import Gripper
from components.intake import Intake


class LedController:
    status_lights: StatusLights
    gripper: Gripper
    intake: Intake
    want: PieceColour
    side: PickupFromSide

    def __init__(self) -> None:
        self.want = PieceColour.NONE
        self.side = PickupFromSide.NONE

    def gripped_piece_is_cone(self) -> bool:
        return False

    def should_place_left(self) -> bool:
        return False

    def wants_cone(self) -> bool:
        return False

    def wants_cube(self) -> bool:
        return False

    def want_cube(self) -> None:
        self.want = PieceColour.CUBE

    def want_cone(self) -> None:
        self.want = PieceColour.CONE

    def place_left(self) -> None:
        self.side = 

    def execute(self) -> None:
        CONE = self.gripper.get_full_closed() and self.gripped_piece_is_cone()
        CUBE = self.intake.is_game_piece_present()
        LEFT = self.should_place_left()
        RIGHT = not self.should_place_left()
        WANTS_CONE = self.wants_cone()
        WANTS_CUBE = self.wants_cube()
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
"""