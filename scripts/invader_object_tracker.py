#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import Point

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread

class ObjectTracker(Thread):
    def __init__(self, map_image: np.ndarray, pixel_threshold: float, distance_threshold: float) -> None:
        """
        pixel_threshold is the threshold for the pixel values in percentage of white
        distance_threshold is how far the centers can be before they're classified as different objects
        """
        super().__init__()
        self.map_image: np.ndarray = map_image
        self.pixel_threshold: float = pixel_threshold
        self.distance_threshold: float = distance_threshold

        self.detected_objects: dict[str, tuple[int, int]] = {}

        self.daemon = True
        self.runnable = self.run

        return None

    def detect_objects(self):
        """
        Returns a list of tuples of the center of each object
        """
        
        # Threshold the image
        # set all pixels below the threshold to 0 and above to 1
        self.map_image = np.array(self.map_image, dtype=np.uint8)
        _, self.map_image = cv.threshold(self.map_image, self.pixel_threshold, 255, cv.THRESH_BINARY)
        contours, _ = cv.findContours(self.map_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        plt.imshow(self.map_image)
        
        polygons = []
        for contour in contours:
            polygon = []
            for point in contour:
                polygon.append(Point(x=float(point[0][0]), y=float(point[0][1])))
            plt.plot(
                [point.x for point in polygon],
                [point.y for point in polygon],
                color="red"
            )

        # STOPED HERE... ONLY CARE ABOUT POLYGONS ON MY LEVEL

        plt.pause(0.001)

        for contour in contours:
            M = cv.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.detected_objects[str((cx, cy))] = contour

    def run(self) -> None:
        self.detect_objects()

        return None