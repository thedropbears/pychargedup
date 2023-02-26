from enum import Enum

from utilities.game import GamePiece

from components.gripper import Gripper
from components.leds import StatusLights

from controllers.acquire_cone import AcquireConeController
from controllers.acquire_cube import AcquireCubeController
from controllers.recover import RecoverController
from controllers.score_game_piece import ScoreGamePieceController


class CurrentController(Enum):
    RECOVER = 0
    ACQUIRE_CONE = 1
    ACQUIRE_CUBE = 2
    SCORE_GAME_PIECE = 3
    NONE = 4


class Behaviour:
    gripper: Gripper
    status_lights: StatusLights

    acquire_cone: AcquireConeController
    acquire_cube: AcquireCubeController
    score_game_piece: ScoreGamePieceController
    recover: RecoverController

    def __init__(self) -> None:
        self.current_controller = CurrentController.NONE
        self.held_piece = GamePiece.NONE

    def execute(self) -> None:
        if (
            self.current_controller == CurrentController.RECOVER
            and not self.recover.is_executing
        ):
            self.current_controller = CurrentController.NONE
        if self.gripper.get_full_closed():
            if self.current_controller == CurrentController.ACQUIRE_CONE:
                self.held_piece = GamePiece.CONE
            if self.current_controller == CurrentController.ACQUIRE_CUBE:
                self.held_piece = GamePiece.CUBE
        else:
            if self.current_controller == CurrentController.SCORE_GAME_PIECE:
                self.held_piece = GamePiece.NONE

    def do_recover(self) -> None:
        self.current_controller = CurrentController.RECOVER
        self.acquire_cone.done()
        self.acquire_cube.done()
        self.score_game_piece.done()
        self.recover.engage()

    def has_no_controller(self) -> bool:
        return self.current_controller == CurrentController.NONE

    def start_acquiring_cone(self) -> None:
        self.current_controller = CurrentController.ACQUIRE_CONE
        self.acquire_cone.engage()

    def target_right_cone(self) -> None:
        self.status_lights.want_cone_right()
        self.acquire_cone.target_right()

    def target_left_cone(self) -> None:
        self.status_lights.want_cone_left()
        self.acquire_cone.target_left()

    def done_acquiring_cone(self) -> None:
        self.acquire_cone.done()

    def start_acquiring_cube(self) -> None:
        self.current_controller = CurrentController.ACQUIRE_CUNE
        self.acquire_cube.engage()
        self.status_lights.want_cube()

    def done_acquiring_cube(self) -> None:
        self.acquire_cube.done()

    def start_scoring_piece(self) -> None:
        self.current_controller = CurrentController.SCORE_GAME_PIECE
        self.score_game_piece.engage()

    def done_scoring_piece(self) -> None:
        self.score_game_piece.done()
