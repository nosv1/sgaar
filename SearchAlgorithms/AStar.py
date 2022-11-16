import matplotlib.pyplot as plt

from .Colors import Colors
from .Grid import Grid
from .Node import Node
from .PathFinder import PathFinder

class AStar(PathFinder):
    def __init__(self, start: Node, goal: Node, grid: Grid) -> None:
        super().__init__(start, goal, grid)

    def add_neighbors_to_open_set(self):
        """
        Adds neighbors of the current node to open set.
        """
        # instead of nested loops, we define a list of valid moves
        diagonal_moves = [
            (-self.grid.grid_spacing, -self.grid.grid_spacing),  # left bottom
            (-self.grid.grid_spacing, self.grid.grid_spacing),   # left top
            (self.grid.grid_spacing, -self.grid.grid_spacing),   # right bottom
            (self.grid.grid_spacing, self.grid.grid_spacing),    # right top
        ]

        move_list = [
            (-self.grid.grid_spacing, 0),                        # left center
            (0, -self.grid.grid_spacing),                        # center bottom
            (0, self.grid.grid_spacing),                         # center top
            (self.grid.grid_spacing, 0),                         # right center
        ]

        if self.do_diagonals:
            move_list += diagonal_moves

        for move in move_list:
            neighbor: Node = Node(
                x=self._current_node.x + move[0],
                y=self._current_node.y + move[1],
                parent=self._current_node
            )

            if not self.grid.node_is_valid(neighbor):
                continue

            neighbor.start_to_node_cost = (
                self._current_node.start_to_node_cost +
                neighbor.distance_to(self._current_node)
            )
            neighbor.heuristic_cost = neighbor.distance_to(self.goal)

            # if we've already noted this neighbor
            if neighbor.id in self._open_set:
                # if its cost got cheaper
                if neighbor.total_cost < self._open_set[neighbor.id].total_cost:
                    self._open_set[neighbor.id] = neighbor

            # brand new neighbor
            elif neighbor.id not in self._closed_set:
                self._open_set[neighbor.id] = neighbor
        

    def find_path(self) -> None:
        # initialize open set with start node
        self._open_set[self.start.id] = self.start
        self._current_node = self.start

        # while we are not at the goal
        while self._current_node != self.goal:
            # add neighbors to open set
            self.add_neighbors_to_open_set()

            # remove current node from open set
            del self._open_set[self._current_node.id]

            # add current node to closed set
            self._closed_set[self._current_node.id] = self._current_node

            # get node from open set with smallest total cost
            self._current_node = min(
                self._open_set.values(),
                key=lambda node: node.total_cost
            )
        
        # update goal cost and parent with current node
        self.goal.start_to_node_cost = (
            self._current_node.start_to_node_cost +
            self._current_node.distance_to(self.goal)
        )
        self.goal.parent = self._current_node

        # get path, looping backwards through the parents
        self._path = [self.goal]
        while self._path[-1] != self.start:
            self._path.append(self._path[-1].parent)