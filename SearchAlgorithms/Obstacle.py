from __future__ import annotations
import math
import numpy as np
import os
import random
import re

from .Node import Node

class Obstacle(Node):
    def __init__(self, x: float, y: float, radius: float) -> None:
        super().__init__(x, y)
        self.radius = radius

        self._bounding_box: dict[str, Node] = {}

    def obstacles_from_file(
        filename: str, radius: float, delimter: str=","
    ) -> dict[str, Obstacle]:
        obstacles: dict[str, Obstacle] = {}
        # cwd: str = os.path.dirname(os.path.realpath(__file__))
        # with open(os.path.join(cwd, filename), "r") as f:
        with open(filename, "r") as f:
            for line in f:
                x, y = line.split(delimter)
                obstacle: Obstacle = Obstacle(
                    x=float(x), y=float(y), radius=radius
                )
                obstacles[obstacle.id] = obstacle
        return obstacles

    def generate_obstacles(
        count: int, 
        radius: float | str, 
        min_x: float, 
        max_x: float, 
        min_y: float, 
        max_y: float
    ) -> dict[str, Obstacle]:
        obstacles: dict[str, Obstacle] = {}
        randomize = type(radius) == str and re.match(r"random\(.*,.*\)", radius)
        randomize_range = tuple((
                            float(n) 
                            for n 
                            in radius.split("(")[1].split(")")[0].split(",")
                        )) if randomize else None

        for i in range(count):
            if randomize:
                radius = random.uniform(*randomize_range)
                
            obstacle: Obstacle = Obstacle(
                x=random.uniform(min_x, max_x),
                y=random.uniform(min_y, max_y),
                radius=radius
            )
            obstacles[obstacle.id] = obstacle

        return obstacles

    def inflate(self, inflation_amount):
        """
        Inflates the obstacle

        :param inflation_amount: amount to inflate the obstacle by
        :return: None
        """
        self.radius += inflation_amount

    def set_bounding_box(self, spacing: float, include_diaganols=True) -> None:
        """
        The nodes surrounding the obstacle are its bounding box

        :param spacing: spacing
        :return: None
        """
        self.radius = math.floor(self.radius / spacing) * spacing
        for x in np.arange(
            self.x - self.radius,
            self.x + self.radius + spacing,
            spacing
        ):
            for y in np.arange(
                self.y - self.radius,
                self.y + self.radius + spacing,
                spacing
            ):
                node: Node = Node(x, y, parent=self)
                if self.is_point_inside_obstacle(node):
                    self._bounding_box[node.id] = node

    def is_point_inside_obstacle(self, node: Node) -> bool:
        """
        Checks if a point is inside the obstacle

        :param node: node to check
        :return: True if the point is inside the obstacle, False otherwise
        """
        distance: float = self.distance_to(node)
        return distance <= self.radius + 0.00001