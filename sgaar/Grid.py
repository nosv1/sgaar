from __future__ import annotations

from array import array
import numpy as np
from numpy import ndarray
from PIL import Image
import yaml

import matplotlib.pyplot as plt

from .Point import Point

from SearchAlgorithms.Grid import Grid as SA_Grid
from SearchAlgorithms.Node import Node
from SearchAlgorithms.Obstacle import Obstacle

class Grid(SA_Grid):
    def __init__(self, /, 
        base_map_pgm_file: str, 
        base_map_yaml_file: str,
        obstacle_radius: float) -> None:
        self.obstacle_radius: float = obstacle_radius
        
        self.grid_spacing = None
        self.base_map: ndarray = self.map_from_pgm(
            base_map_pgm_file, base_map_yaml_file)
        self.obstacles: dict[str, Obstacle] = self.obstacles_from_map(self.base_map)
        self.min_x = 0
        self.min_y = 0
        self.max_x = self.base_map.shape[0] * self.grid_spacing
        self.max_y = self.base_map.shape[1] * self.grid_spacing
            
        super().__init__(
            self.min_x, 
            self.max_x, 
            self.min_y, 
            self.max_y, 
            self.grid_spacing, 
            self.obstacles)

        return None

    def obstacles_from_map(self, map: ndarray) -> dict[str, Node]:
        """  Returns a dictionary of snapped to grid invalid nodes from a map """

        # white is 255, grey is 205, black is 0
        obstacles_array = np.where(map <= 205)  # [[x], [y]]

        obstacles: dict[str, Obstacle] = {}
        for i in range(len(obstacles_array[0])):
            point = Point(
                obstacles_array[0][i] * self.grid_spacing,
                obstacles_array[1][i] * self.grid_spacing,
                0)
            point.snap_to_grid(self.grid_spacing)
            obstacle = Obstacle(point.x, point.y, self.obstacle_radius)
            obstacles[obstacle.id] = obstacle
        
        return obstacles

    def map_from_pgm(self, pgm_file_path: str, yaml_file_path: str) -> ndarray:
        """  Reads a map from a file and returns it as a numpy array """

        with open(yaml_file_path, 'r') as yaml_file:
            # image: /home/justin/final_map.pgm
            # mode: trinary
            # resolution: 0.05
            # origin: [-3.45, -3.39, 0]
            # negate: 0
            # occupied_thresh: 0.65
            # free_thresh: 0.25

            yaml_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
            origin: Point(
                x=yaml_data['origin'][0], 
                y=yaml_data['origin'][1], 
                z=yaml_data['origin'][2])
            resolution: float = yaml_data['resolution']  # m / pixel
            occupancy_threshold: float = yaml_data['occupied_thresh']
            free_threshold: float = yaml_data['free_thresh']
            self.grid_spacing = resolution

        pgm_map = np.asarray(Image.open(pgm_file_path))
        return pgm_map