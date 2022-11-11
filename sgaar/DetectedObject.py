import numpy as np

from .Point import Point

class DetectedObject:
    def __init__(self, distance: float, angle: float, current_position: Point, current_yaw: float):
        self.distance = distance
        self.angle = angle  # (-π, π) radians

        self.point: Point = self.calculate_2d_point(current_position, current_yaw)

    def __str__(self):
        return f"Range: {self.distance}, Angle: {self.angle}"

    def calculate_2d_point(self, current_position: Point, current_yaw: float):
        x = self.distance * np.cos(self.angle)
        y = self.distance * np.sin(self.angle)
        z=0

        return Point(
            x=current_position.x + x,
            y=current_position.y + y,
            z=z
        )
