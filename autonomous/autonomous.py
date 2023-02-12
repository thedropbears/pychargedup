from controllers.scoring import ScoringController
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d
from utilities.game import (
    GamePiece,
    get_staged_pieces,
    field_flip_pose2d,
    field_flip_rotation2d,
    BLUE_NODES,
)
import wpilib


class CubeAutoBase:
    """Auto which starts with a preloaded cone, scores it then drives to pickup and score cubes"""

    scoring: ScoringController
    chassis: Chassis

    def __init__(self):
        self.start_pose = Pose2d(4, 3, 0)
        # list of cube idx and angle to hit pickup at as if was on blue alliance
        # 0 is closest to wall, 3 is closest to substations
        self.cubes: list[tuple[int, Rotation2d]] = []
        # list of nodes to score on, 0 is closest to wall, 8 is closest to substations
        self.nodes: list[int] = []
        # TODO: impliment or rule out balancing during auto
        self.balance_at_end = False
        self.finished = False

    def on_enable(self):
        if self.scoring.is_red():
            start_pose = field_flip_pose2d(self.start_pose)
        else:
            start_pose = self.start_pose
        self.chassis.set_pose(start_pose)

        self.scoring.wants_piece = GamePiece.CUBE
        self.scoring.is_holding = GamePiece.CONE
        self.scoring.cube_queue = []
        all_pieces = get_staged_pieces(wpilib.DriverStation.getAlliance())
        for idx, rotation in self.cubes:
            position = all_pieces[idx]
            # flip angle, position is already correctly flipped
            angle = (
                field_flip_rotation2d(rotation) if self.scoring.is_red() else rotation
            )
            # approach angle is the same as chassis heading
            pose = Pose2d(position, angle)
            self.scoring.cube_queue.append((pose, angle))

        self.scoring.score_queue = self.nodes

    def on_iteration(self, tm: float):
        if not self.finished:
            self.scoring.autodrive = True
            self.scoring.engage()
        if (
            len(self.scoring.cube_queue) == 0
            and self.scoring.is_holding is GamePiece.NONE
        ):
            self.finished = True

    def on_disable(self):
        self.scoring.wants_piece = GamePiece.CONE


class WallSide3(CubeAutoBase):
    MODE_NAME = "Wall side 3"

    def setup(self):
        self.start_pose = Pose2d(1.88, BLUE_NODES[0][0].y, 0)
        self.cubes = [(0, Rotation2d(0)), (1, Rotation2d.fromDegrees(40))]
        self.nodes = [0, 1, 1]


class SubstationSide3(CubeAutoBase):
    MODE_NAME = "Substation side 3"
    DEFAULT = True

    def setup(self):
        self.start_pose = Pose2d(1.88, BLUE_NODES[0][8].y, 0)
        self.cubes = [(3, Rotation2d(0)), (2, Rotation2d.fromDegrees(-40))]
        self.nodes = [8, 7, 7]
