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

        self.heading_PID = heading_PID
        self.throttle_PID = throttle_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

        self.point_cloud: dict[str, Point] = {}
        self.grid_spacing: float = self.max_speed / 5  # lidar hz is about 5 times a second

    def on_odom_callback(self) -> None:
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        if self.sim_elapsed_time > 5:
            self.twist.angular.z = self.max_turn_rate
        
        self.twist.linear.x = self.max_speed
        self.move()

        return None

    def on_lidar_callback(self) -> None:
        if not self.detected_objects:
            return

        for detected_object in self.detected_objects:
            detected_object_key = (detected_object
                .point
                .snap_to_grid(self.grid_spacing)
                .key())
            if detected_object_key not in self.point_cloud:
                self.point_cloud[detected_object_key] = detected_object.point

        return None

    def on_occupancy_grid_callback(self) -> None:
        # create 2d np array of size x, y
        map_data = np.array(
            [[0 for x in range(self.map_meta_data.width)] 
            for y in range(self.map_meta_data.height)])
        height = self.map_meta_data.height
        width = self.map_meta_data.width
        # loop self.map of type array
        for i in range(len(self.map)):
            map_data[int(i / width)][i % width] = self.map[i]
            print(map_data[int(i / width)][i % width], end=", ")
            if (i % width == 0):
                print()

        return None

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()

        # elif self.last_callback == self.__lidar_callback:
        #     self.on_lidar_callback()

        # elif self.last_callback == self.__occupancy_grid_callback:
        #     self.on_occupancy_grid_callback()

        return None


def main() -> None:
    rclpy.init()

    tester: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        namespace='',
        name="Tester")

    tester.throttle_PID = None
    tester.max_speed = 0.22
    tester.max_turn_rate = 1.4
    # pursuer.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
         
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    scenario = Scenario()
    scenario.start = Node(x=1.2, y=1.7)
    scenario.goal = Node(x=1.6, y=4.0)

    pgm_file_path = "/home/thomas/ros2_ws/src/sgaar/slam_map/final_map.pgm"
    yaml_file_path = "/home/thomas/ros2_ws/src/sgaar/slam_map/final_map.yaml"
    print('Building grid...')
    grid = Grid(base_map_pgm_file=pgm_file_path, base_map_yaml_file=yaml_file_path, obstacle_radius=0.01)
    print('\rBuilding grid... Done')

    scenario.obstacles = grid.obstacles
    scenario.grid = grid

    print("Setting up plot...")
    fig, ax = plt.subplots()
    fig.set_size_inches(8, 8)
    
    # set colors
    fig.patch.set_facecolor(Colors.grey)
    ax.set_facecolor(Colors.grey)
    ax.title.set_color(Colors.light_grey)
    ax.xaxis.label.set_color(Colors.light_grey)
    ax.yaxis.label.set_color(Colors.light_grey)

    # set axis labels
    ax.tick_params(axis='x', colors=Colors.light_grey)
    ax.tick_params(axis='y', colors=Colors.light_grey)

    # set title
    # ax.set_title(
    #     f"{scenario.algorithm.__class__.__name__}\n"
    #     f"Time: TBD, Cost: TBD"
    # )

    # set axis limits
    ax.set_xlim(
        scenario.grid.min_x - scenario.grid.grid_spacing * 2, 
        scenario.grid.max_x + scenario.grid.grid_spacing * 2
    )
    ax.set_ylim(
        scenario.grid.min_y - scenario.grid.grid_spacing * 2, 
        scenario.grid.max_y + scenario.grid.grid_spacing * 2
    )

    print('Inflating obstacles...')
    grid.inflate_obstacles(0.02)
    print("Setting nodes...")
    grid.set_nodes()
    print('\rSetting nodes... Done')

    scenario.algorithm = AStar(grid=grid, start=scenario.start, goal=scenario.goal)
    scenario.algorithm.stopwatch.start()
    scenario.algorithm.find_path()
    scenario.algorithm.stopwatch.stop()
    print(scenario.algorithm.stopwatch.elapsed_time)
    scenario.plot_obstacles(ax, Colors.red)
    scenario.plot_start_and_goal(ax)
    scenario.plot_path(ax, Colors.light_blue)
    plt.show()

    while rclpy.ok():
        rclpy.spin_once(tester)
        tester.update()

        if degrees(tester.roll) > 1:
            break

        # if round(tester.sim_elapsed_time) % 10 == 0:
        #     print(tester.sim_elapsed_time)
        #     tester.dump_point_cloud(filename=f"{tester.name}_point_cloud.csv")

    tester.close_logs()
    tester.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()