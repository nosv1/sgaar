from __future__ import annotations

from .Grid import Grid
from .Node import Node
from .Stopwatch import Stopwatch

class PathFinder:
    def __init__(
        self, 
        start: Node, 
        goal: Node, 
        grid: Grid, 
        do_diagonals: bool = True
    ) -> None:
        self.start = start
        self.goal = goal
        self.grid = grid
        self.do_diagonals = do_diagonals

        self._current_node: Node = None
        self._open_set: dict[str, Node] = {}
        self._closed_set: dict[str, Node] = {}
        self._path: list[Node] = []

        self.stopwatch: Stopwatch = Stopwatch()

    @property
    def path(self) -> list[Node]:
        return self._path

    @property
    def open_set(self) -> dict[str, Node]:
        return self._open_set

    @property
    def closed_set(self) -> dict[str, Node]:
        return self._closed_set