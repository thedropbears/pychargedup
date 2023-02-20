from controllers.scoring import ScoringController, ScoreMovement, PickupMovement
from components.chassis import Chassis
from wpimath.geometry import Rotation2d, Translation2d
from utilities.game import (
    GamePiece,
    Node,
    Rows,
)


class CubeAutoBase:
    """Auto which starts with a preloaded cone, scores it then drives to pickup and score cubes"""

    scoring: ScoringController
    chassis: Chassis

    def __init__(self, cubes: list[PickupMovement], nodes: list[ScoreMovement]) -> None:
        self.pickup_list = cubes
        self.score_list = nodes
        self.finished = False

    def on_enable(self) -> None:
        start_pose, _ = self.scoring.score_location_from_node(self.score_list[0].node)
        self.chassis.set_pose(start_pose)

        self.scoring.wants_piece = GamePiece.CUBE
        self.scoring.is_holding = GamePiece.CONE
        self.scoring.cube_stack = self.pickup_list[::-1]
        self.scoring.score_stack = self.score_list[::-1]

    def on_iteration(self, tm: float) -> None:
        if not self.finished:
            self.scoring.autodrive = True
            self.scoring.engage()
        if len(self.scoring.cube_stack) == 0 and len(self.scoring.score_stack) == 0:
            self.finished = True

    def on_disable(self) -> None:
        self.scoring.wants_piece = GamePiece.CONE


class WallSide3(CubeAutoBase):
    MODE_NAME = "Wall side 3"

    def __init__(self):
        super().__init__(
            cubes=[
                PickupMovement(0, Rotation2d(0), []),
                PickupMovement(1, Rotation2d.fromDegrees(40), []),
            ],
            nodes=[
                ScoreMovement(Node(row=Rows.HIGH, col=0), []),
                ScoreMovement(Node(row=Rows.HIGH, col=1), []),
                ScoreMovement(Node(row=Rows.MID, col=1), []),
            ],
        )


class SubstationSide3(CubeAutoBase):
    MODE_NAME = "Substation side 3"
    DEFAULT = True

    def __init__(self):
        super().__init__(
            cubes=[
                PickupMovement(3, Rotation2d.fromDegrees(-10), []),
                PickupMovement(2, Rotation2d.fromDegrees(-40), []),
            ],
            nodes=[
                ScoreMovement(Node(row=Rows.HIGH, col=8), []),
                ScoreMovement(Node(row=Rows.HIGH, col=7), []),
                ScoreMovement(Node(row=Rows.MID, col=7), [Translation2d(5, 4.5)]),
            ],
        )
