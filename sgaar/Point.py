from __future__ import annotations

from math import atan2, pi

class Point:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x: float = x
        self.y: float = y
        self.z: float = z

    def __str__(self) -> str:
        return f"[{self.x:.1f}, {self.y:.1f}, {self.z:.1f}]"

    def __eq__(self, other: Point) -> bool:
        return (
            self.x == other.x and 
            self.y == other.y and 
            self.z == other.z
        )
        
    def key(self, round_to: int = 3) -> str:
        return f"{self.x:.{round_to}f},{self.y:.{round_to}f},{self.z:.{round_to}f}"

    def distance_to(self, other: Point) -> float:
        dx: float = self.x - other.x
        dy: float = self.y - other.y
        dz: float = self.z - other.z
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    def heading_to(self, other: Point) -> float:
        dx: float = other.x - self.x
        dy: float = other.y - self.y
        heading: float = atan2(dy, dx)
        return heading if heading >= 0 else heading + 2 * pi

    def snap_to_grid(self, grid_spacing: float, round_up=True) -> Point:
        if round_up:
            x: float = grid_spacing * round(self.x / grid_spacing)
            y: float = grid_spacing * round(self.y / grid_spacing)
            z: float = grid_spacing * round(self.z / grid_spacing)
        else:
            x: float = grid_spacing * (self.x // grid_spacing)
            y: float = grid_spacing * (self.y // grid_spacing)
            z: float = grid_spacing * (self.z // grid_spacing)
        return Point(x, y, z)