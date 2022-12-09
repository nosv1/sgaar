#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import Point

import numpy as np
from threading import Thread

class AStarPoint(Point):
    def __init__(
        self, 
        x: float, 
        y: float, 
        g_score: float, 
        f_score: float, 
        parent: AStarPoint = None, 
        radius: float = 0.0,
        heading: float =  None) -> None:
        super().__init__(x=x, y=y)
        self.g_score = g_score  # distance from start
        self.f_score = f_score  # distance from start + distance to goal
        self.parent = parent
        self.radius = radius    # for when you want to ignore obstacles around a point
        self.heading = heading  # if not None, used to determine which neighbors are available

    @property
    def key(self):
        return (int(self.x), int(self.y))

    def __eq__(self, other):
        return self.key == other.key

    def distance_to(self, other: Point):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class AStar:
    def __init__(self, start: Point, goal: AStarPoint, map_image: np.ndarray, grid_spacing: float, step_size: float = 1):
        super().__init__()
        self.start: AStarPoint = AStarPoint(start.x, start.y, 0, 0)
        self.goal: AStarPoint = goal
        self.map_image = map_image
        self.grid_spacing = grid_spacing
        self.step_size = step_size

        self.current_point: AStarPoint = self.start
        self.open_set: dict[str, AStarPoint] = {self.current_point.key: self.current_point}
        self.closed_set: dict[str, AStarPoint] = {}
        self.path: list[AStarPoint] = []
        
        self.move_list: list[tuple[float]] = self.set_move_list()

        return None

    def set_move_list(self) -> list[tuple[float]]:
        # rotate around an origin at 1 degree separation, snapping to the grid

        moves: list[tuple[float]] = []
        for i in range(0, 360, 6):
            move = (
                self.step_size * np.cos(i * np.pi / 180),
                self.step_size * np.sin(i * np.pi / 180)
            )
            move = (
                int(move[0] / self.grid_spacing) * self.grid_spacing,
                int(move[1] / self.grid_spacing) * self.grid_spacing
            )
            moves.append(move)

        return moves

    def add_neighbors_to_open_set(self):

        last_move = (float('inf'), float('inf'))
        for move in self.move_list:
            if move == last_move:
                continue
            last_move = move

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
                # if neighbor.distance_to(self.goal) > self.goal.radius:
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