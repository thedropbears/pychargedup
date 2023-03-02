from dataclasses import dataclass
from enum import Enum, auto
import wpilib
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Translation3d
import robotpy_apriltag
from components.chassis import Chassis

apriltag_layout = robotpy_apriltag.loadAprilTagLayoutField(
    robotpy_apriltag.AprilTagField.k2023ChargedUp
)

FIELD_WIDTH = 8.0161
tag_8 = apriltag_layout.getTagPose(8)
tag_1 = apriltag_layout.getTagPose(1)
# for type checker
assert tag_8 is not None and tag_1 is not None
FIELD_LENGTH = tag_1.x + tag_8.x


class GamePiece(Enum):
    """Enum for tracking scored pieces, held pieces and node pieces"""

    CUBE = auto()
    CONE = auto()
    BOTH = auto()
    NONE = auto()


class Rows(Enum):
    HIGH = 0
    MID = 1
    LOW = 2


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_translation3d(t: Translation3d):
    return Translation3d(FIELD_LENGTH - t.x, t.y, t.z)


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, t.y)


Y_OFFSET_TO_GRID: float = 0.512
Y_DISTANCE_BETWEEN_NODES: float = 0.5588

SCORING_Z_LOOK_UP = {
    # heights to cube node base
    GamePiece.CUBE: {Rows.HIGH: 0.826, Rows.MID: 0.522, Rows.LOW: 0.0},
    # heights to top of pole
    GamePiece.CONE: {Rows.HIGH: 1.170, Rows.MID: 0.865, Rows.LOW: 0.0},
    GamePiece.BOTH: {Rows.LOW: 0.0},
}
SCORING_X_LOOK_UP = {Rows.HIGH: 0.36, Rows.MID: 0.79, Rows.LOW: 1.18}

RED_NODES: list[list[Translation3d]] = []
BLUE_NODES: list[list[Translation3d]] = []
for row in Rows:
    BLUE_NODES.append([])
    RED_NODES.append([])
    for col in range(9):
        if row == Rows.LOW:
            piece = GamePiece.BOTH
        elif col % 3 == 1:
            piece = GamePiece.CUBE
        else:
            piece = GamePiece.CONE

        y = Y_OFFSET_TO_GRID + Y_DISTANCE_BETWEEN_NODES * col
        blue_node = Translation3d(
            SCORING_X_LOOK_UP[row], y, SCORING_Z_LOOK_UP[piece][row]
        )
        BLUE_NODES[-1].append(blue_node)
        RED_NODES[-1].append(field_flip_translation3d(blue_node))


@dataclass
class Node:
    row: Rows
    col: int

    def get_valid_piece(self) -> GamePiece:
        if row is Rows.LOW:
            return GamePiece.BOTH
        if col % 2 == 1:
            return GamePiece.CUBE
        else:
            return GamePiece.CONE


def get_node_location(node: Node) -> Translation3d:
    if is_red():
        return RED_NODES[node.row.value][node.col]
    else:
        return BLUE_NODES[node.row.value][node.col]


# edge of the hybrid node baffles
GRIDS_EDGE_X = 1.37


def get_score_location(node: Node) -> tuple[Pose2d, Rotation2d]:
    """Returns the goal and approach direction for the given node"""
    red_side = is_red()
    node_location = get_node_location(node)

    # always be up against the grids
    x = GRIDS_EDGE_X + Chassis.LENGTH / 2 + 0.1
    blue_pose = Pose2d(x, node_location.y, Rotation2d(0))
    goal = field_flip_pose2d(blue_pose) if red_side else blue_pose
    approach_blue = Rotation2d.fromDegrees(180)
    approach = field_flip_rotation2d(approach_blue) if red_side else approach_blue
    return goal, approach


def get_closest_node(pos: Translation2d, piece: GamePiece, row: Rows) -> Node:
    def get_node_dist(node: Node) -> float:
        return get_score_location(node)[0].translation().distance(pos)

    if piece == GamePiece.CONE:
        nodes = [Node(row, i) for i in range(9) if i % 3 != 1]
    elif piece == GamePiece.CUBE:
        nodes = [Node(row, i) for i in range(1, 9, 3)]
    else:  # NONE or BOTH
        nodes = [Node(row, i) for i in range(9)]

    return min(nodes, key=get_node_dist)


# tag in blue loading bay, on red side of field 16=x
tag_4 = apriltag_layout.getTagPose(4)
assert tag_4 is not None
blue_substation_tag = tag_4.translation()

# tag in red loading bay, on the blue side of field 0=x
tag_5 = apriltag_layout.getTagPose(5)
assert tag_5 is not None
red_substation_tag = tag_5.translation()

# offset from tag to the left/right cone spots
# aligned with the second vertical pole underneath the shelf on each side
SUBSTATION_Y_OFFSET = 0.722

# side closer to the wall
DOUBLE_SUBSTATION_RED_WALL = red_substation_tag + Translation3d(
    0, SUBSTATION_Y_OFFSET, 0
)
DOUBLE_SUBSTATION_BLUE_WALL = blue_substation_tag + Translation3d(
    0, SUBSTATION_Y_OFFSET, 0
)

# side closer to grids
DOUBLE_SUBSTATION_RED_GRID = red_substation_tag - Translation3d(
    0, SUBSTATION_Y_OFFSET, 0
)
DOUBLE_SUBSTATION_BLUE_GRID = blue_substation_tag - Translation3d(
    0, SUBSTATION_Y_OFFSET, 0
)


def get_double_substation(left_side: bool) -> Translation3d:
    """Left/Right sides from perspective of current drivers"""
    if is_red():
        if left_side:
            return DOUBLE_SUBSTATION_RED_GRID
        else:
            return DOUBLE_SUBSTATION_RED_WALL
    else:
        if left_side:
            return DOUBLE_SUBSTATION_BLUE_WALL
        else:
            return DOUBLE_SUBSTATION_BLUE_GRID


SINGLE_SUBSTATION_RED = Translation2d(2.303, 8.077)
SINGLE_SUBSTATION_BLUE = field_flip_translation2d(SINGLE_SUBSTATION_RED)


def get_single_substation(alliance: wpilib.DriverStation.Alliance) -> Translation2d:
    if alliance == wpilib.DriverStation.Alliance.kBlue:
        return SINGLE_SUBSTATION_BLUE
    else:
        return SINGLE_SUBSTATION_RED


STAGED_PIECES_Y_OFFSET = 0.920
STAGED_PIECES_Y_BETWEEN = 1.219
STAGED_PIECES_X = 4.5  # 7.067

STAGED_PIECES_BLUE: list[Translation2d] = []
STAGED_PIECES_RED: list[Translation2d] = []
for i in range(4):
    blue_piece = Translation2d(
        STAGED_PIECES_X, STAGED_PIECES_Y_OFFSET + STAGED_PIECES_Y_BETWEEN * i
    )
    STAGED_PIECES_BLUE.append(blue_piece)
    STAGED_PIECES_RED.append(field_flip_translation2d(blue_piece))


def get_staged_pieces(
    alliance: Optional[wpilib.DriverStation.Alliance] = None,
) -> list[Translation2d]:
    if alliance is None:
        alliance = get_team()
    if alliance == wpilib.DriverStation.Alliance.kBlue:
        return STAGED_PIECES_BLUE
    else:
        return STAGED_PIECES_RED


def is_red() -> bool:
    return get_team() == wpilib.DriverStation.Alliance.kRed


def get_team() -> wpilib.DriverStation.Alliance:
    return wpilib.DriverStation.getAlliance()


def get_cone_pickup(targeting_left: bool, offset_x: float) -> tuple[Pose2d, Rotation2d]:
    """Returns the goal and approach direction for the given side"""
    red_side = is_red()
    goal_rotation = Rotation2d(0) if red_side else Rotation2d.fromDegrees(180)
    goal_approach = Rotation2d.fromDegrees(180) if red_side else Rotation2d(0)
    cone_trans = get_double_substation(targeting_left).toTranslation2d()
    goal_trans = cone_trans + Translation2d(-offset_x if red_side else offset_x, 0)

    goal = Pose2d(goal_trans, goal_rotation)

    return (
        goal,
        goal_approach,
    )


def get_staged_pickup(
    staged_idx: int, rotation: Rotation2d, distance: float = 0.3
) -> tuple[Pose2d, Rotation2d]:
    """Gets the pose to go to to pick up a staged piece
    Args:
        staged_idx: index of the piece, 0 is closest to the wall, 3 is closest to the loading zones
        rotation: approach dir and chassis rotation
        distance: distance from robot center to piece
    """
    piece = get_staged_pieces()[staged_idx]
    robot_pose = Pose2d(
        piece.x - rotation.cos() * distance,
        piece.y - rotation.sin() * distance,
        rotation,
    )
    return robot_pose, rotation
