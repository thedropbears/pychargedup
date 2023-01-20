from dataclasses import dataclass
from enum import Enum
import numpy as np
import numpy.typing


class FieldSide(Enum):
    RED = 0
    BLUE = 1


class GamePiceType(Enum):
    CONE = 1
    CUBE = 2
    BOTH = 3


@dataclass
class NodeLocation:
    column: int
    row: int
    side: FieldSide

    def get_index(self):
        return self.column + self.row * 9 + self.side.value * 27

    def get_allowed_piece(self) -> GamePiceType:
        if self.row == 0:
            return GamePiceType.BOTH
        if self.column % 3 == 1:
            return GamePiceType.CUBE
        else:
            return GamePiceType.CONE


class ScoreTracker:

    control_loop_wait_time: float
    # the confidence required to count a possible piece as existing
    CONFIDENCE_THRESHOLD = 0.5

    # the weight increase per second
    SIGHTING_TRUE_WEIGHT = 0.5
    # the weight decrease per second
    SIGHTING_FALSE_WEIGHT = -0.01
    PLACE_WEIGHT = 5

    def __init__(self) -> None:
        # confidences for all pieces on the field, including other side
        self.confidences: np.ndarray = np.zeros(3 * 3 * 3 * 2)

    def add_vision(self, node_location: NodeLocation, node_value: bool) -> None:
        """
        Modify the confidence of the piece in location for node_id
        node_id: The id of the node
        node_value: Weather there is a piece there
        """
        idx = node_location.get_index()
        cur_confidence = self.confidences[idx]
        if node_value and cur_confidence < 1:
            self.confidences[idx] += (
                self.SIGHTING_TRUE_WEIGHT * self.control_loop_wait_time
            )
        if not node_value:
            self.confidences[idx] += (
                self.SIGHTING_FALSE_WEIGHT * self.control_loop_wait_time
            )

    def get_row(self, row: int) -> np.typing.NDArray[np.bool_]:
        return self.confidences[row * 9 : (row + 1) * 9] > self.CONFIDENCE_THRESHOLD

    def add_piece(self, node_location: NodeLocation) -> None:
        """
        A function to add a piece as 100% on position of layer of node_id
        node_id: The node (1-8)
        layer: The layer (0-2)
        position: The postion (0-2)
        """
        self.confidences[node_location.get_index()] = self.PLACE_WEIGHT

    @staticmethod
    def evaluate_row(row_values: np.typing.NDArray[np.bool_], row: int) -> float:
        """
        Evaluate how good a state of a row is
        row_values: bool array or if there are pieces in nodes
        row: which row (0 bottom, 1 middle, 2 top)
        """
        total_score = 0.0
        piece_value = 0
        if row == 0:
            piece_value = 2
        if row == 1:
            piece_value = 3
        if row == 2:
            piece_value = 5

        # Count points for scoring pieces
        for col in row_values:
            if col:
                total_score += piece_value
        # Count points for scoring link
        cur_row_idx = 0
        while cur_row_idx <= 6:
            # np.bool_ dosent support adding
            n = 0
            n += 1 if row_values[cur_row_idx] else 0
            n += 1 if row_values[cur_row_idx + 1] else 0
            n += 1 if row_values[cur_row_idx + 2] else 0
            if n == 3:
                total_score += 5
                cur_row_idx += 3
                continue
            if n == 2:
                total_score += 0.1
            cur_row_idx += 1

        return total_score

    def can_place_piece(self, location: NodeLocation, piece: GamePiceType) -> bool:
        """Checks if a piece is allowed to be placed in a node"""
        return (
            location.get_allowed_piece() == piece
            or location.get_allowed_piece() == GamePiceType.BOTH
        )

    def evaluate_place_location(
        self, location: NodeLocation, piece_type: GamePiceType
    ) -> float:
        """
        Give a reletive score of how good it is to score in this position,
        roughly equivilant to the score after placing.
        location: Where to place the piece
        piece_type: What piece
        """
        if not self.can_place_piece(location, piece_type):
            return -1

        # get turn confidences into bools
        row = self.get_row(location.row)
        row[location.column] = True
        return self.evaluate_row(row, location.row)

    def get_node_picklist(self, piece_type: GamePiceType) -> list[NodeLocation]:
        """Gets a sorted list of which locations to put piece_type"""
        # capture the piece type in a function to sort with
        def evaluate_func(location):
            return self.evaluate_place_location(location, piece_type)

        all_locations: list[NodeLocation] = []
        for side in (FieldSide.BLUE, FieldSide.RED):
            for row in range(2):
                for col in range(9):
                    all_locations.append(NodeLocation(col, row, side))

        return sorted(all_locations, key=evaluate_func, reverse=True)
