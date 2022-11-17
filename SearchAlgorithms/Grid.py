from __future__ import annotations

import numpy as np

from .Node import Node
from .Obstacle import Obstacle

class Grid:
    def __init__(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        grid_spacing: float,
        obstacles: dict[str, Obstacle],
    ) -> None:
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.grid_spacing = grid_spacing
        self.obstacles = obstacles

        self._valid_nodes: dict[str, Node] = {}
        self._invalid_nodes: dict[str, Node] = {}

    @property
    def nodes(self) -> dict[str, Node]:
        """
        combines the valid and invalid nodes in the grid

        :return: combined nodes
        """
        return {**self._valid_nodes, **self._invalid_nodes}

    def inflate_obstacles(self, inflation_amount: float):
        """
        Inflates the obstacles in the grid

        :param inflation_amount: amount to inflate the obstacles by
        """
        for _id, obstacle in self.obstacles.items():
            obstacle.inflate(inflation_amount)

    def inflate_bounds(self, inflation_amount: float):
        """
        Inflates the bounds of the grid

        :param inflation_amount: amount to inflate the bounds by
        """
        self.min_x += self.grid_spacing * round(inflation_amount / self.grid_spacing)
        self.max_x -= self.grid_spacing * round(inflation_amount / self.grid_spacing)
        self.min_y += self.grid_spacing * round(inflation_amount / self.grid_spacing)
        self.max_y -= self.grid_spacing * round(inflation_amount / self.grid_spacing)

    def node_in_bounds(self, node: Node) -> bool:
        """
        Checks if a node is in the bounds of the grid

        :param node: node to check
        :return: True if the node is in the bounds of the grid, False otherwise
        """
        return (
            self.min_x <= node.x <= self.max_x
            and self.min_y <= node.y <= self.max_y
        )

    def node_in_obstacle(self, node: Node) -> bool:
        if node.id in self._invalid_nodes:
            # invalid_node could be an obstacle's bounding box or 
            # node outside of the grid
            invalid_node: Node = self._invalid_nodes[node.id]

            # check if invalid node has a parent
            if invalid_node.parent:

                # if it does, we deduce it's an obstacle's bounding box
                obstacle: Obstacle = self.obstacles[invalid_node.parent.id]
                if obstacle.is_point_inside_obstacle(node):
                    return True

            else:
                return True

        return False

    def node_is_valid(self, node: Node) -> bool:
        """
        Checks if a node is valid

        :param node: node to check
        :return: True if the node is valid, False otherwise
        """
        return (
            self.node_in_bounds(node) and
            not self.node_in_obstacle(node)
        )

    def set_nodes(self):
        """
        Sets the valid and invalid nodes in the grid
        """
        for _id, obstacle in self.obstacles.items():
            obstacle.set_bounding_box(self.grid_spacing)

            self._invalid_nodes[obstacle.id] = obstacle
            for _id, invalid_node in obstacle._bounding_box.items():
                self._invalid_nodes[_id] = invalid_node

        # looping in bound nodes
        for row in np.arange(
            self.min_x, self.max_x + self.grid_spacing, self.grid_spacing
        ):
            for col in np.arange(
                self.min_y, self.max_y + self.grid_spacing, self.grid_spacing
            ):
                node = Node(
                    x=row,
                    y=col
                )

                if node.id not in self._invalid_nodes:
                    self._valid_nodes[node.id] = node

    def snap_node_to_grid(self, node: Node) -> Node:
        """
        Snaps a node to the grid

        :param node: node to snap
        :return: snapped node
        """
        node.x = round(node.x / self.grid_spacing) * self.grid_spacing
        node.y = round(node.y / self.grid_spacing) * self.grid_spacing
        return node
