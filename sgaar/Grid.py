from array import array
import numpy as np
from numpy import ndarray
from PIL import Image
import yaml

import matplotlib.pyplot as plt

from .Point import Point

class Grid:
    def __init__(self, /, 
        base_map_pgm_file: str, 
        base_map_yaml_file: str, 
        grid_spacing: float) -> None:
        """ grid_spacing comes from MapMetaData.resolution """
        self.base_map = self.map_from_pgm(base_map_pgm_file, base_map_yaml_file)
        self.grid_spacing = grid_spacing
        self.invalid_nodes: list[tuple] = []

        return None

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

        pgm_map = np.asarray(Image.open(pgm_file_path))
        obstacles = np.where(pgm_map <= 205)
        # plot obstacles
        plt.scatter(obstacles[1], obstacles[0])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        return pgm_map