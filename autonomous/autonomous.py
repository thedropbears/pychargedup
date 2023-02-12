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
    Chassis: Chassis

    def __init__(self):
        self.start_pose = Pose2d(4, 3, 0)
        # list of cube idx and angle to hit pickup at as if was on blue alliance
        # 0 is closest to wall, 3 is closest to substations
        self.cubes: list[tuple[int, Rotation2d]] = []
        # TODO: impliment or rule out balancing during auto
        self.balance_at_end = False

    def on_enable(self):
        if self.scoring.is_red():
            start_pose = field_flip_pose2d(self.start_pose)
        else:
            start_pose = self.start_pose
        self.Chassis.set_pose(start_pose)

        self.scoring.wants_piece = GamePiece.CUBE
        self.scoring.is_holding = GamePiece.CONE
        self.scoring.cube_queue = []
        all_pieces = get_staged_pieces(wpilib.DriverStation.getAlliance())
        for idx, _angle in self.cubes:
            position = all_pieces[idx]
            # flip angle, position is already correctly flipped
            angle = field_flip_rotation2d(_angle) if self.scoring.is_red() else _angle
            # approach angle is the same as chassis heading
            pose = Pose2d(position, angle)
            self.scoring.cube_queue.append((pose, angle))

    def on_iteration(self):
        self.scoring.autodrive = True
        self.scoring.engage()


class WallSide3(CubeAutoBase):
    MODE_NAME = "Wall side 3"
    DEFAULT = True

    def __init__(self):
        self.start_pose = Pose2d(1.88, BLUE_NODES[0][0].y, 0)
        self.cubes = [(0, Rotation2d(0)), (1, Rotation2d.fromDegrees(40))]
