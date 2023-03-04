from magicbot import feedback
from wpimath.geometry import Pose2d
from utilities.game import Y_OFFSET_TO_GRID, Y_DISTANCE_BETWEEN_NODES

import math


class StartPoseTracker:
    DECAY_FACTOR = 0.99

    def __init__(self):
        self.cone_rows = [col for col in range(9) if col % 3 != 1]
        self.cone_ys = [
            Y_OFFSET_TO_GRID + Y_DISTANCE_BETWEEN_NODES * col for col in self.cone_rows
        ]
        self.accs = [0.0 for _ in self.cone_rows]

    def execute(self) -> None:
        pass

    def add_measurement(self, pose: Pose2d) -> None:
        y = pose.translation().y
        min_d = math.inf
        min_i = 0
        for i in range(len(self.cone_ys)):
            d = abs(self.cone_ys[i] - y)
            if d < min_d:
                min_d = d
                min_i = i
        self.accs[min_i] += 1.0
        for i in range(len(self.accs)):
            self.accs[i] *= self.DECAY_FACTOR

    @feedback
    def best_row(self) -> int:
        max_a = -1.0
        max_i = 0
        for i in range(len(self.accs)):
            if self.accs[i] > max_a:
                max_a = self.accs[i]
                max_i = i
        return self.cone_rows[max_i]
