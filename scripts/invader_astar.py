#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import Point

import numpy as np
from threading import Thread

class AStarPoint(Point):
    def __init__(
        self, x: float, 
        y: float, 
        g_score: float, 
        f_score: float, 
        parent: AStarPoint = None, 
        radius: float = 0.0) -> None:
        super().__init__(x=x, y=y)
        self.g_score = g_score  # distance from start
        self.f_score = f_score  # distance from start + distance to goal
        self.parent = parent
        self.radius = radius    # for when you want to ignore obstacles around a point

    @property
    def key(self):
        return (int(self.x), int(self.y))

    def __eq__(self, other):
        return self.key == other.key

    def distance_to(self, other: Point):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class AStar:
    def __init__(self, start: Point, goal: AStarPoint, map_image: np.ndarray, step_size: float = 1):
        super().__init__()
        self.start: AStarPoint = AStarPoint(start.x, start.y, 0, 0)
        self.goal: AStarPoint = goal
        self.map_image = map_image
        self.step_size = step_size

        self.current_point: AStarPoint = self.start
        self.open_set: dict[str, AStarPoint] = {self.current_point.key: self.current_point}
        self.closed_set: dict[str, AStarPoint] = {}
        self.path: list[AStarPoint] = []
        
        diaganol_leg_distance = np.sqrt(0.5 * self.step_size**2)
        self.move_list = [
            (-diaganol_leg_distance, -diaganol_leg_distance),  # top left
            (-diaganol_leg_distance, diaganol_leg_distance),   # bottom left
            (diaganol_leg_distance, -diaganol_leg_distance),   # top right
            (diaganol_leg_distance, diaganol_leg_distance),    # bottom right
            (0, -self.step_size),  # up
            (0, self.step_size),   # down
            (-self.step_size, 0),  # left
            (self.step_size, 0),   # right
        ]

    def add_neighbors_to_open_set(self):

        for move in self.move_list:
            neighbor: AStarPoint = AStarPoint(
                self.current_point.x + move[0],
                self.current_point.y + move[1],
                0,
                0,
                parent=self.current_point
            )

            if (min(neighbor.x, neighbor.y) < 0             # if neighbor is out of bounds
                or neighbor.y >= self.map_image.shape[0]    # if neighbor is out of bounds
                or neighbor.x >= self.map_image.shape[1]):  # if neighbor is out of bounds
                continue

            if self.map_image[int(neighbor.y)][int(neighbor.x)]:  # if neighbor is a wall
                if neighbor.distance_to(self.goal) > self.goal.radius:
                    continue
            
            if neighbor.key in self.closed_set:
                continue

            neighbor.g_score = (self.current_point.g_score
                + self.current_point.distance_to(neighbor))
            neighbor.f_score = (neighbor.g_score
                + neighbor.distance_to(self.goal))

            if neighbor.key in self.open_set:
                if neighbor.f_score < self.open_set[neighbor.key].f_score:
                    self.open_set[neighbor.key] = neighbor

            else:
                self.open_set[neighbor.key] = neighbor

        return None
        
    def plan_path(self):
        while self.current_point.distance_to(self.goal) > self.step_size:
        # while self.current_point != self.goal:
            self.add_neighbors_to_open_set()
            del self.open_set[self.current_point.key]
            self.closed_set[self.current_point.key] = self.current_point
            self.current_point = min(self.open_set.values(), key=lambda p: p.f_score)
            
        self.goal.f_score = self.current_point.f_score
        self.goal.parent = self.current_point.parent

        self.path = [self.goal]
        while self.path[-1].parent is not None:
            self.path.append(self.path[-1].parent)