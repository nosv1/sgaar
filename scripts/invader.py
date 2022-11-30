#!/usr/bin/env python3

from __future__ import annotations

# python imports
from ament_index_python.packages import get_package_share_directory
import json
from math import degrees, pi
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
import cv2

# ros2 imports
import rclpy

# personal imports
from post_processing.Colors import Colors
from sgaar.Logger import Logger
from sgaar.math_tools import clamp
from sgaar.Grid import Grid
from sgaar.PID import PID
from sgaar.Point import Point
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
        
        return None

    def on_odom_callback(self) -> None:
        return
    
    def on_occupancy_grid_callback(self) -> None:
        plot self.map
        fig, ax = plt.subplots()
        # mirror map
        self.map = np.flip(self.map, axis=0)
        # rotate map 90 degrees
        self.map = np.rot90(self.map, k=1, axes=(0, 1))
        threshold = 90
        ax.imshow(self.map > threshold, cmap='gray')
        plt.show()

        return None
    
    def on_local_footprint_callback(self) -> None:
        fig, ax = plt.subplots()

        for point in self.local_costmap_footprint.polygon.points:
            ax.scatter(point.x, point.y, c='r')
        
        plt.show()

    def update(self) -> None:

        print(self.last_callback)

        # if self.last_callback == self.__odom_callback:
        #     self.on_odom_callback()

        if self.last_callback == self.__occupancy_grid_callback:
            self.on_occupancy_grid_callback()
        
        if self.last_callback == self.__local_costmap_footprint_callback:
            self.on_local_footprint_callback()

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

    for subscription in invader.subscriptions:
        if subscription.topic_name == f"{invader.namespace}/odom":
            invader.destroy_subscription(subscription)
         
        elif subscription.topic_name == f"{invader.namespace}/clock":
            invader.destroy_subscription(subscription)

    while rclpy.ok():
        rclpy.spin_once(invader)
        invader.update()

        if degrees(invader.roll) > 1:
            break

        # if round(tester.sim_elapsed_time) % 10 == 0:
        #     print(tester.sim_elapsed_time)
        #     tester.dump_point_cloud(filename=f"{tester.name}_point_cloud.csv")

    invader.close_logs()
    invader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()