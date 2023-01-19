from typing import Optional


class ScoreTracker:

    # the confidence required to count a possible piece as existing
    CONFIDENCE_THRESHOLD = 50

    # the weight given to a vision input that states a piece as there
    TRUE_WEIGHT = 100
    # the weight given to a vision input that states a piece as not there
    FALSE_WEIGHT = 0

    # a type alias to represent confidence of pieces being on a node (basically a 3x3 grid of floats)
    NodeConfidence = tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]

    # a type alias to represent confidence of pieces being on a node for inputting data. this relies on a `True` being 80% confidence and `False` being 20% confidence
    NodeConfidenceBoolean = tuple[
        tuple[bool, bool, bool], tuple[bool, bool, bool], tuple[bool, bool, bool]
    ]

    # a type alias to help with inputting confidence scores
    # \/ Node ID   , \/ node confidence
    Input = tuple[int, NodeConfidence | NodeConfidenceBoolean]

    def __init__(self) -> None:
        NodeConfidence = tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ]
        # idk why but i made everything but node ids start at 0, so we need to add 9 None's here to not break node #8
        self.confidences: list[NodeConfidence] = [
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0), (0, 0, 0)),
        ]  # mf i hope this hard typed shit is worth it

        self.known_pieces: dict[int, dict[int, dict[int, bool]]] = {}

    def add_vision(self, _confidences: Input, _type: str = "score") -> None:
        """
        Modify the confidence of pieces being in locations for _confidences[0]
        _confidences: The info on the nodeId and confidence data
        _type: The type of confidence (can be "score" or "boolean")
        """
        if _type == "score":
            confidences_in = _confidences[1]
            confidences = self.confidences[_confidences[0]]
            for i in range(len(confidences)):
                for j in range(len(confidences[i])):
                    if self.known_pieces[_confidences[0]][i][j]:
                        break  # we know this piece is there
                    if confidences[i] is not None:
                        confidences[i][j] = (confidences[i][j] + confidences_in[i][j]) / 2  # type: ignore
                    else:
                        confidences[i][j] = confidences_in[i][j]  # type: ignore
            self.confidences[_confidences[0]] = confidences
        elif _type == "boolean":
            confidences_in = _confidences[1]
            confidences = self.confidences[_confidences[0]]
            for i in range(0, 2):
                for j in range(0, 2):
                    if self.known_pieces[_confidences[0]][i][j]:
                        break  # we know this piece is there
                    if confidences[i] is not None:
                        confidences[i][j] = (confidences[i][j] + self.TRUE_WEIGHT if confidences_in[i][j] else self.FALSE_WEIGHT) / 2  # type: ignore
                    else:
                        confidences[i][j] = self.TRUE_WEIGHT if confidences_in[i][j] else self.FALSE_WEIGHT  # type: ignore
            self.confidences[_confidences[0]] = confidences

    def add_piece(self, node_id: int, layer: int, position: int) -> None:
        """
        A function to add a piece as 100% on position of layer of node_id
        node_id: The node (1-8)
        layer: The layer (0-2)
        position: The postion (0-2)
        """
        self.known_pieces[node_id][layer][position] = True
        self.confidences[node_id][layer][position] = 100  # type: ignore

    def indentify_link_possibility(
        self, layer: tuple[float, float, float], piece: str
    ) -> Optional[int]:
        if (
            layer[0] > self.CONFIDENCE_THRESHOLD
            and layer[1] > self.CONFIDENCE_THRESHOLD
            and layer[2] > self.CONFIDENCE_THRESHOLD
        ):
            # the layer is full
            return None
        if (
            layer[0] > self.CONFIDENCE_THRESHOLD
            and layer[1] > self.CONFIDENCE_THRESHOLD
            and piece == "cone"
        ):
            return 2
        if (
            layer[0] > self.CONFIDENCE_THRESHOLD
            and layer[2] > self.CONFIDENCE_THRESHOLD
            and piece == "cube"
        ):
            return 1
        if (
            layer[1] > self.CONFIDENCE_THRESHOLD
            and layer[2] > self.CONFIDENCE_THRESHOLD
            and piece == "cone"
        ):
            return 0
        # there is no link possibility
        return None

    def identify_placement_possibility(
        self, layer: tuple[float, float, float], piece: str
    ) -> Optional[int]:
        if (
            layer[0] > self.CONFIDENCE_THRESHOLD
            and layer[1] > self.CONFIDENCE_THRESHOLD
            and layer[2] > self.CONFIDENCE_THRESHOLD
        ):
            # the layer is full
            return None
        if not (layer[0] > self.CONFIDENCE_THRESHOLD) and piece == "cone":
            return 0
        if not (layer[1] > self.CONFIDENCE_THRESHOLD) and piece == "cube":
            return 1
        if not (layer[2] > self.CONFIDENCE_THRESHOLD) and piece == "cone":
            return 2
        # the piece cannot be placed anywhere
        return None

    def get_best(self, node_id: int, piece: str) -> Optional[tuple[int, int]]:
        """
        Get the best position to place a piece on in node `node_id`

        1) Check for possible links
            a) On the top line
            b) On the middle line
            c) On the bottom line
        2) Check for possible scores
            a) On the top line
            b) On the middle line
            c) On the bottom line

        node_id: The node ID that we want to put the piece on
        piece: The type of piece we want to place onto the node, can be either "cone" or "cube"
        Returns (level, placement on the level)
        """

        #  LINK STEP  #

        # prioritize links at the top
        link_top = self.indentify_link_possibility(self.confidences[node_id][0], piece)
        if link_top is not None:
            return (0, link_top)  # type: ignore
        # next link at the middle
        link_mid = self.indentify_link_possibility(self.confidences[node_id][1], piece)
        if link_mid is not None:
            return (1, link_mid)  # type: ignore
        # last, link at the bottom
        link_floor = self.indentify_link_possibility(
            self.confidences[node_id][2], piece
        )
        if link_floor is not None:
            return (1, link_floor)  # type: ignore

        # SINGLE SCORE CHECK #

        # prioritize pieces at the top
        place_top = self.identify_placement_possibility(
            self.confidences[node_id][0], piece
        )
        if place_top is not None:
            return (0, place_top)  # type: ignore
        # next, place at the middle
        place_mid = self.identify_placement_possibility(
            self.confidences[node_id][1], piece
        )
        if place_mid is not None:
            return (1, place_mid)  # type: ignore
        # last, place at the bottom
        place_floor = self.identify_placement_possibility(
            self.confidences[node_id][2], piece
        )
        if place_floor is not None:
            return (2, place_floor)  # type: ignore

        # there is no-where to place the piece on this node
        return None

    def get(self, piece: str) -> Optional[tuple[int, int, int]]:
        """
        Get the best position to place a piece on all the known nodes (in terms of points)

        1) Check for possible links
            a) On the top line
            b) On the middle line
            c) On the bottom line
        2) Check for possible scores
            a) On the top line
            b) On the middle line
            c) On the bottom line

        piece: The type of piece we want to place onto the node, can be either "cone" or "cube"
        Returns (node, level, placement on the level)
        """

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # prioritize links at the top
            link_top = self.indentify_link_possibility(self.confidences[i][0], piece)
            if link_top is not None:
                return (i, 0, link_top)  # type: ignore

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # next link at the middle
            link_mid = self.indentify_link_possibility(self.confidences[i][1], piece)
            if link_mid is not None:
                return (i, 1, link_mid)  # type: ignore

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # last, link at the bottom
            link_floor = self.indentify_link_possibility(self.confidences[i][2], piece)
            if link_floor is not None:
                return (i, 1, link_floor)  # type: ignore

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # prioritize pieces at the top
            place_top = self.identify_placement_possibility(
                self.confidences[i][0], piece
            )
            if place_top is not None:
                return (i, 0, place_top)  # type: ignore

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # next, place at the middle
            place_mid = self.identify_placement_possibility(
                self.confidences[i][1], piece
            )
            if place_mid is not None:
                return (i, 1, place_mid)  # type: ignore

        for i in range(0, len(self.confidences)):
            if self.confidences[i] is None:
                break
            # last, place at the bottom
            place_floor = self.identify_placement_possibility(
                self.confidences[i][2], piece
            )
            if place_floor is not None:
                return (i, 2, place_floor)  # type: ignore

        return None
