#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import Point

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

import pyvisgraph as vg
from pyvisgraph.visible_vertices import polygon_crossing

class ObjectTracker:
    def __init__(self, start_position: Point, map_image: np.ndarray, pixel_threshold: float, distance_threshold: float) -> None:
        """
        pixel_threshold is the threshold for the pixel values in percentage of white
        distance_threshold is how far the centers can be before they're classified as different objects
        """
        super().__init__()
        self.start_position = start_position
        self.map_image: np.ndarray = map_image
        self.pixel_threshold: float = pixel_threshold
        self.distance_threshold: float = distance_threshold

        self.detected_objects: dict[str, tuple[int, int]] = {}

        return None

    def detect_objects(self):
        """
        Returns a list of tuples of the center of each object
        """
        
        # Threshold the image

        contours, hierarchy = cv.findContours(self.map_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 2)

        # polygons = []
        # for contour in contours:
        #     poly = []
        #     for i, edge in enumerate(contour):
        #         poly.append(vg.Point(edge[0][0], edge[0][1]))
        #     # connect the last point to the first point
        #     # poly.append(vg.Point(contour[-1][0][1], contour[0][0][0]))
        #     polygons.append(poly)

        # if self.start_position != Point():
        #     self.parent_polygon_index = -1

        #     g = vg.VisGraph()
        #     try:
        #         g.build(polygons, workers=4)
        #     except ZeroDivisionError:
        #         return
        #     except UnboundLocalError:
        #         return

        #     for polygon in g.graph.polygons:
        #         if polygon_crossing(vg.Point(self.start_position.x, self.start_position.y), g.graph.polygons[polygon]):
        #             if polygon > self.parent_polygon_index:
        #                 self.parent_polygon_index = polygon
                        
        #     for i in range(len(polygons) - 1, -1, -1):
        #         if i != self.parent_polygon_index:
        #             if hierarchy[0][i][3] > self.parent_polygon_index + 1:
        #                 del polygons[i]

        plt.imshow(self.map_image)

        # plot the edges of the contours
        # plot the centers of the contours
        for contour in contours:
            plt.plot(contour[:, 0, 0], contour[:, 0, 1], 'k-')
            plt.plot(np.mean(contour[:, 0, 0]), np.mean(contour[:, 0, 1]), 'ro')
