from controllers.scoring import ScoringController
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d
from utilities.game import (
    GamePiece,
    Node,
    Rows,
    get_staged_pieces,
    field_flip_pose2d,
    field_flip_rotation2d,
    is_red,
)
import wpilib


class CubeAutoBase:
    """Auto which starts with a preloaded cone, scores it then drives to pickup and score cubes"""

    scoring: ScoringController
    chassis: Chassis

    def __init__(self, cubes: list[tuple[int, Rotation2d]], nodes: list[Node]) -> None:
        # list of cube idx and angle to hit pickup at as if was on blue alliance
        # 0 is closest to wall, 3 is closest to substations
        self.cubes = cubes
        # list of nodes to score on, 0 is closest to wall, 8 is closest to substations
        self.nodes = nodes
        self.finished = False

    def on_enable(self) -> None:
        (start_pose, _), _ = self.scoring.score_location_from_node(self.nodes[0], False)
        if is_red():
            start_pose = field_flip_pose2d(start_pose)
        else:
            start_pose = start_pose
        self.chassis.set_pose(start_pose)

        self.scoring.wants_piece = GamePiece.CUBE
        self.scoring.is_holding = GamePiece.CONE
        self.scoring.cube_stack = []
        all_pieces = get_staged_pieces(wpilib.DriverStation.getAlliance())
        for idx, rotation in self.cubes[::-1]:
            position = all_pieces[idx]
            # flip angle, position is already correctly flipped
            angle = (
                field_flip_rotation2d(rotation) if is_red() else rotation
            )
            # approach angle is the same as chassis heading
            pose = Pose2d(position, angle)
            self.scoring.cube_stack.append((pose, angle))

        self.scoring.score_stack = self.nodes[::-1]

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
            cubes=[(0, Rotation2d(0)), (1, Rotation2d.fromDegrees(40))],
            nodes=[
                Node(row=Rows.HIGH, col=0),
                Node(row=Rows.HIGH, col=1),
                Node(row=Rows.MID, col=1),
            ],
        )


class SubstationSide3(CubeAutoBase):
    MODE_NAME = "Substation side 3"
    DEFAULT = True

    def __init__(self):
        super().__init__(
            cubes=[(3, Rotation2d(0)), (2, Rotation2d.fromDegrees(-40))],
            nodes=[
                Node(row=Rows.HIGH, col=8),
                Node(row=Rows.HIGH, col=7),
                Node(row=Rows.MID, col=7),
            ],
        )
