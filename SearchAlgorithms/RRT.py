import matplotlib.pyplot as plt
import numpy as np
import random

from .Colors import Colors
from .Node import Node
from .PathFinder import PathFinder

class RRT(PathFinder):
    def __init__(
        self, 
        step_length: float, 
        sub_step_length: float=None,
        *args, **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        self.step_length = step_length
        self.sub_step_length = (
            step_length / 5 
            if not sub_step_length 
            else sub_step_length
        )

    def generate_random_node(self) -> Node:
        x_range_expansion: float = (self.grid.max_x - self.grid.min_x) / 4
        y_range_expansion: float = (self.grid.max_y - self.grid.min_y) / 4
        self._random_node = Node(
            x=random.uniform(
                self.grid.min_x - x_range_expansion,
                self.grid.max_x + x_range_expansion
            ),
            y=random.uniform(
                self.grid.min_y - y_range_expansion,
                self.grid.max_y + y_range_expansion
            )
        )
        return self._random_node

    def step_towards_node(self, root: Node, node: Node, step_length: float) -> Node:
        """
        Move step_length towards node from root.
        """
        node: Node = root.step_towards_node(node, step_length)
        node.start_to_node_cost = root.start_to_node_cost + step_length
        node.parent = root
        return node

    def is_valid_step(
        self, start: Node, stop: Node, sub_step_length: float
    ) -> bool:
        """
        Check if points of sub-step length between start and stop are valid.
        """
        for step in np.linspace(0, 1, int(start.distance_to(stop) / sub_step_length)):
            node: Node = Node(
                x=start.x + step * (stop.x - start.x),
                y=start.y + step * (stop.y - start.y)
            )
            snapped_node: Node = self.grid.snap_node_to_grid(node)
            if not self.grid.node_is_valid(snapped_node):
                return False
        return True

    def find_path(self) -> None:
        """
        While current node is not in reach of goal
            - Generate random node
            - Find closest discovered node
            - Step towards random node from closest node
            - If step and sub-steps are valid
                * add to open set
        """
        self._open_set[self.start.id] = self.start
        self._current_node = self.start

        # while current node is not in reach of goal
        while self._current_node.distance_to(self.goal) > self.step_length:
            while True:
                # generate random node
                random_node: Node = self.generate_random_node()

                # find closest discovered node
                closest_node: Node = random_node.find_closest_node(
                    self._open_set.values()
                )

                # step towards random node from closest node
                self._current_node = self.step_towards_node(
                    closest_node, random_node, self.step_length
                )

                ## snap node to grid
                # snapped_node: Node = self.grid.snap_node_to_grid(
                    # Node(self._current_node.x, self._current_node.y)
                # )
                # if self.grid.node_is_valid(snapped_node):

                # check if step is valid
                if self.is_valid_step(
                    closest_node, self._current_node, self.sub_step_length
                ):
                    self._open_set[self._current_node.id] = self._current_node
                    # ax.plot(
                    #     random_node.x, 
                    #     random_node.y,
                    #     marker="o",
                    #     color=Colors.orange,
                    #     markersize=4
                    # )
                    # ax.plot(
                    #     closest_node.x,
                    #     closest_node.y,
                    #     marker="o",
                    #     color=Colors.light_blue,
                    #     markersize=4
                    # )
                    # ax.plot(
                    #     self._current_node.x,
                    #     self._current_node.y,
                    #     marker="o",
                    #     color=Colors.green,
                    #     markersize=4
                    # )
                    # plt.pause(0.001)
                    break

        # update goal's cost and parent
        self.goal.start_to_node_cost = (
            self._current_node.start_to_node_cost + 
            self._current_node.distance_to(self.goal)
        )
        self.goal.parent = self._current_node

        # construct path
        self._path = [self.goal]
        while self._path[-1].parent:
            self._path.append(self._path[-1].parent)