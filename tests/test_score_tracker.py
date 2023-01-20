import numpy as np
from components.score_tracker import ScoreTracker, GamePiceType, NodeLocation, FieldSide

EMPTY_ROW = np.array([False, False, False, False, False, False, False, False, False])
FULL_ROW = np.array([True, True, True, True, True, True, True, True, True])


def test_evaluate_row_empty():
    for i in range(9):
        new_row = EMPTY_ROW.copy()
        new_row[i] = True
        # check if we prefer to place a piece
        assert ScoreTracker.evaluate_row(new_row, 0) > ScoreTracker.evaluate_row(
            EMPTY_ROW, 0
        )
        assert ScoreTracker.evaluate_row(new_row, 1) > ScoreTracker.evaluate_row(
            EMPTY_ROW, 1
        )
        assert ScoreTracker.evaluate_row(new_row, 2) > ScoreTracker.evaluate_row(
            EMPTY_ROW, 2
        )
        # check if we prefer higher rows
        assert ScoreTracker.evaluate_row(new_row, 1) > ScoreTracker.evaluate_row(
            new_row, 0
        )
        assert ScoreTracker.evaluate_row(new_row, 2) > ScoreTracker.evaluate_row(
            new_row, 1
        )


def test_evaluate_row_link():
    row_with_link = np.array(
        [True, True, True, False, False, False, False, False, False]
    )
    row_without_link = np.array(
        [True, True, False, True, False, False, False, False, False]
    )
    row_with_2_links = np.array(
        [True, True, True, True, True, True, False, False, False]
    )

    for row_idx in (0, 1, 2):
        assert (
            ScoreTracker.evaluate_row(row_without_link, row_idx)
            < ScoreTracker.evaluate_row(row_with_link, row_idx)
            < ScoreTracker.evaluate_row(row_with_2_links, row_idx)
        )


def test_get_node_picklist():
    # fmt: off
    row_with_possible_link = np.array(
        [0, 0, 0,  0, 0, 0,  0, 0, 0,
         1, 0, 1,  0, 0, 0,  0, 0, 0,
         0, 0, 0,  0, 0, 0,  0, 0, 0]
    )
    # fmt: on

    tracker = ScoreTracker()
    tracker.confidences = row_with_possible_link

    out = tracker.get_node_picklist(GamePiceType.CUBE)

    assert out[0] == NodeLocation(1, 1, FieldSide.BLUE)


def test_get_node_picklist_where_link_cannot_be_completed_by_item():
    # fmt: off
    row_with_possible_link = np.array(
        [0, 0, 0,  0, 0, 0,  0, 0, 0,
         1, 0, 1,  0, 0, 0,  0, 0, 0,
         0, 0, 0,  0, 0, 0,  0, 0, 0]
    )
    # fmt: on

    tracker = ScoreTracker()
    tracker.confidences = row_with_possible_link

    out = tracker.get_node_picklist(GamePiceType.CONE)

    assert out[0] == NodeLocation(3, 1, FieldSide.BLUE)
