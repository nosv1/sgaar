#!/usr/bin/env python3

from __future__ import annotations

# python imports
from ament_index_python.packages import get_package_share_directory
import cv2
import json
from math import ceil, degrees, pi
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
import pyvisgraph as vg
from pyvisgraph.visible_vertices import point_in_polygon, polygon_crossing
from pyvisgraph.graph import Point as vg_Point

# ros2 imports
import rclpy
from geometry_msgs.msg import Point

# personal imports
from post_processing.Colors import Colors
from sgaar.Logger import Logger
from sgaar.math_tools import clamp
from sgaar.Grid import Grid
from sgaar.PID import PID
from sgaar.Point import Point as sgaar_Point
from sgaar.quaternion_tools import euler_from_quaternion
from sgaar.TurtleNode import Turtle as TurtleNode

from SearchAlgorithms.AStar import AStar
from SearchAlgorithms.Node import Node
from SearchAlgorithms.Scenario import Scenario

class Turtle(TurtleNode):
    def __init__(self, 
        heading_PID: PID, 
        throttle_PID: PID, 
        max_speed: float, 
        max_turn_rate: float,
        **kwargs) -> None:
        super().__init__(**kwargs)

        self.start_position: Point = Point()
        self.parent_polygon_index: int = None
        
        return None

    def on_odom_callback(self) -> None:
        return
    
    def on_global_costmap_subscriber(self) -> None:

        # mirror map
        # self.map_image = np.flip(self.map_image, axis=0)
        # #rotate map 90 degrees
        # self.map_image = np.rot90(self.map_image, k=1, axes=(0, 1))
        threshold = 95
        plt.clf()
        plt.imshow(self.map_image > threshold, cmap='gray', )

        if self.amcl_position != Point():
            plt.scatter(self.amcl_position.x / -0.05, self.amcl_position.y / -0.05, c='r', marker='x')
        
        if self.start_position == Point():
            self.start_position = Point( 
                x=(self.map_origin.x - self.amcl_position.x) / -0.05, 
                y=(self.map_origin.y - self.amcl_position.y) / -0.05, 
                z=0.0)

        img = np.array(self.map_image, dtype=np.uint8)
        #make 3 channels
        img = np.stack((img, img, img), axis=2)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(img, threshold, 255, 0)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 2)
        print(hierarchy)

        polygons = []
        for contour in contours:
            poly = []
            for point in contour:
                poly.append(vg.Point(point[0][0], point[0][1]))
            polygons.append(poly)
             
        # construct a visibility graph
        g = vg.VisGraph()
        g.build(polygons)  # can use workers=4 for multiprocessing

        # i = len(polys) - 1
        # while i >= 0:
        #     poly = polys[i]
        #     for point in poly:
        #         containing_poly = g.point_in_polygon(point)
        #         print(containing_poly)
        #         if containing_poly != poly and containing_poly != -1:
        #             del polys[i]
        #             break
        #     i -=1


        if self.start_position != Point():
            # for i, polygon in enumerate(polygons):
            #     parent_polgyon = polygon_crossing(
            #         vg_Point(self.start_position.x, self.start_position.y), 
            #         g.graph.polygons[polygon])

            #     if parent_polgyon != -1:
            #         self.parent_polygon_index = i

            # HACK don't touch :)
            self.parent_polygon_index = 1
        
            for i in range(len(polygons) - 1, 0, -1):
                if i != self.parent_polygon_index:
                    if hierarchy[0][i][3] > self.parent_polygon_index + 1:
                        del polygons[i]

        g = vg.VisGraph()
        g.build(polygons)

        # plot contours
        for poly in polygons:
             plt.plot([p.x for p in poly], [p.y for p in poly], c='r')

             plt.plot([poly[-1].x, poly[0].x], [poly[-1].y, poly[0].y], c='r')

        # use visgraph to find a path
        shortest: list[vg.Point] = g.shortest_path(
            vg.Point(self.amcl_position.x / -0.05, self.amcl_position.y / -0.05),
            vg.Point((self.map_origin.x - 4) / -0.05, (self.map_origin.y - -1.5) / -0.05))
        # shortest.insert(2, vg.Point(6.5, 6.5))
        # shortest.insert(4, vg.Point(8.5, 9.5))
        plt.plot(
            [point.x for point in shortest],
            [point.y for point in shortest],
            color="#ffff00", linewidth=2)
       
        plt.pause(0.05)

        return None
    
    def on_amcl_pose_subscriber(self) -> None:
        # plot amcl pose
        plt.scatter(self.amcl_position.x / - 0.05, self.amcl_position.y / -0.05, c='r', marker='x')
        return None

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()

        if self.last_callback == self.__global_costmap_callback:
            self.on_global_costmap_subscriber()
        
        if self.last_callback == self.__amcl_pose_callback:
            self.on_amcl_pose_subscriber()

        return None


def main():
    rclpy.init()

    invader: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.22,  # max speed
        max_turn_rate=1.4,  # matches constriant for 0.3 m radius turning at max speed
        namespace='',
        name="Invader")

    while rclpy.ok():
        rclpy.spin_once(invader)

        invader.update()

        # if degrees(invader.roll) > 1:
        #     break

        # if round(tester.sim_elapsed_time) % 10 == 0:
        #     print(tester.sim_elapsed_time)
        #     tester.dump_point_cloud(filename=f"{tester.name}_point_cloud.csv")

    invader.close_logs()
    invader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()