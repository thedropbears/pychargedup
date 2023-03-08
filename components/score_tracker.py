from enum import Enum
import numpy as np
import numpy.typing as npt
import wpilib
import magicbot
from ntcore import NetworkTableInstance


class GridNode(Enum):
    CUBE = 0
    CONE = 1
    HYBRID = 2


class ScoreTracker:
    CUBE_MASK = np.array(
        [
            [0, 1, 0, 0, 1, 0, 0, 1, 0],
            [0, 1, 0, 0, 1, 0, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
        ],
        dtype=bool,
    )
    CONE_MASK = np.array(
        [
            [1, 0, 1, 1, 0, 1, 1, 0, 1],
            [1, 0, 1, 1, 0, 1, 1, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1],
        ],
        dtype=bool,
    )

    CONF_EXP_FILTER_ALPHA = 0.8

    CONF_THRESHOLD = 0.2

    def __init__(self) -> None:
        # -1.0 - no piece for certain, 1.0 - a piece for certain, 0.0 - unsure
        self.confidences_red: np.ndarray = np.zeros((3, 9), dtype=float)
        self.confidences_blue: np.ndarray = np.zeros((3, 9), dtype=float)
        self.state_red = np.zeros((3, 9), dtype=bool)
        self.state_blue = np.zeros((3, 9), dtype=bool)
        self.did_states_change = magicbot.will_reset_to(False)
        self.inst = NetworkTableInstance.getDefault()
        nt = self.inst.getTable("left_cam")
        self.nodes = nt.getEntry("nodes")

    def execute(self) -> None:
        if not self.nodes.exists():
            print("skipping")
            return
        _data = self.nodes.getStringArray([])
        data = self.nt_data_to_node_data(_data)
        for node in data:
            side = (
                wpilib.DriverStation.Alliance.kBlue
                if node[0] >= 27
                else wpilib.DriverStation.Alliance.kRed
            )
            col = node[0] % 9
            row = (node[0] % 27) // 9
            self.add_vision_data(
                side=side,
                pos=np.array([row, col]),
                confidence=(1.0 if node[1] else -0.5),
            )

    def nt_data_to_node_data(self, data: list[str]) -> list[tuple[int, bool]]:
        nodes: list[tuple[int, bool]] = []
        for node in data:
            as_array = str(node)
            a = (int(f"{as_array[0]}{as_array[1]}"), as_array[2] == "1")
            nodes.append(a)
        return nodes

    def add_vision_data(
        self, side: wpilib.DriverStation.Alliance, pos: npt.ArrayLike, confidence: float
    ) -> None:
        confidences = (
            self.confidences_red if side == side.kRed else self.confidences_blue
        )
        confidences[pos] = confidences[
            pos
        ] * ScoreTracker.CONF_EXP_FILTER_ALPHA + confidence * (
            1.0 - ScoreTracker.CONF_EXP_FILTER_ALPHA
        )
        if abs(confidences[pos]) > ScoreTracker.CONF_THRESHOLD:
            self.did_states_change = True
            state = self.state_red if side == side.kRed else self.state_blue
            state[pos] = confidence > 0.0

    @staticmethod
    def count_links(r: npt.NDArray[bool]) -> int:
        i = 0
        n = 0
        l = len(r)
        while i < l - 2:
            if r[i] and r[i + 1] and r[i + 2]:
                n += 1
                i += 3
                continue
            i += 1
        return n

    @staticmethod
    def evaluate_state(a: npt.NDArray[bool]) -> int:
        return (
            sum(ScoreTracker.count_links(r) for r in a) * 5
            + a[0].sum() * 5
            + a[1].sum() * 3
            + a[2].sum() * 2
        )

    @staticmethod
    def run_lengths_mod3(state: npt.NDArray[bool]) -> npt.NDArray[int]:
        """
        Returns an array where corresponding in shape to the input, where
        every value is replaced by the length of the longest uninterrupted
        run of true values containing it, modulo 3
        """
        run_lengths = np.zeros_like(state, dtype=int)
        for y in range(3):
            x = 0
            while x < 9:
                if not state[y, x]:
                    x += 1
                    continue
                acc = 0
                for xn in range(x, 9):
                    if not state[y, xn]:
                        break
                    acc += 1
                run_lengths[y, x : x + acc] = acc % 3
                x += acc
        return run_lengths

    @staticmethod
    def get_in_row(arr: npt.NDArray, x: int, y: int, def_val):
        if x < 0 or x > 8:
            return def_val
        else:
            return arr[y, x]

    @staticmethod
    def get_best_moves(
        state: npt.NDArray[bool],
        type_to_test: GridNode,
        link_preparation_score: float = 2.5,
    ) -> npt.NDArray:
        vals = np.zeros_like(state, dtype=float)
        run_lengths = ScoreTracker.run_lengths_mod3(state)
        for y in range(3):
            for x in range(9):
                if (
                    state[y, x]
                    or (
                        type_to_test == GridNode.CUBE
                        and not ScoreTracker.CUBE_MASK[y, x]
                    )
                    or (
                        type_to_test == GridNode.CONE
                        and not ScoreTracker.CONE_MASK[y, x]
                    )
                ):
                    continue
                val = [5.0, 3.0, 2.0][y]
                # Check link completion
                if (
                    ScoreTracker.get_in_row(run_lengths, x - 1, y, 0)
                    + ScoreTracker.get_in_row(run_lengths, x + 1, y, 0)
                    >= 2
                ):
                    val += 5.0
                # Otherwise, check link preparation (state where a link can be completed after 1 move)
                else:
                    for o in [-2, -1, 1, 2]:
                        if ScoreTracker.get_in_row(run_lengths, x + o, y, 0) == 1:
                            val += link_preparation_score
                            break
                vals[y, x] = val
        m = vals.max()
        return np.argwhere(vals == m)
