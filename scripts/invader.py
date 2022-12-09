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
from threading import Thread

# ros2 imports
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData

# personal imports
from invader_astar import AStar, AStarPoint
from invader_object_tracker import ObjectTracker

from PythonRobotics.PathPlanning.DubinsPath import dubins_path_planner as dpp
from post_processing.Colors import Colors
from sgaar.Logger import Logger
from sgaar.math_tools import clamp
from sgaar.Grid import Grid
from sgaar.PID import PID
from sgaar.Point import Point as sgaar_Point
from sgaar.quaternion_tools import euler_from_quaternion
from sgaar.TurtleNode import Turtle as TurtleNode

def yaw_from_points(p1: vg.Point, p2: vg.Point) -> float:
    return np.arctan2(p2.y - p1.y, p2.x - p1.x)

def heading_to(p1: Point, p2: Point) -> float:
    dx: float = p2.x - p1.x
    dy: float = p2.y - p1.y
    heading: float = np.arctan2(-dy, dx)
    return heading if heading >= 0 else heading + 2 * pi

def distance_to(p1: Point, p2: Point) -> float:
    dx: float = p1.x - p2.x
    dy: float = p1.y - p2.y
    return np.sqrt(dx ** 2 + dy ** 2)

def world_to_pixel_coordinates(point: Point, origin: Point, map_meta_data: MapMetaData) -> Point:
    origin_in_pixels: Point = Point(
        x=-origin.x / map_meta_data.resolution,
        y=(origin.y / map_meta_data.resolution) + map_meta_data.height)

    point_in_pixels: Point = Point(
        x=point.x / map_meta_data.resolution + origin_in_pixels.x,
        y=origin_in_pixels.y - (point.y / map_meta_data.resolution))
    
    return Point(
        x=float(int(point_in_pixels.x)),
        y=float(int(point_in_pixels.y)))

# ⠀⠀⠀⠀⠀⠀⢀⣀⣤⣤⣤⣤⣤⣤⣤⣤⣤⣄⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⢀⡴⠋⠀⢀⡤⣖⡫⠭⠭⠭⠭⣍⣑⣒⣒⣒⣻⠭⢝⠲⣄⡀⠀⠀⠀
# ⠀⠀⠀⢀⡾⠁⠠⢖⡽⣪⡑⠬⡭⠭⢙⡄⠀⠀⠀⢐⠒⠒⠒⢤⠀⠀⠹⡄⠀⠀
# ⠀⠀⣠⡞⠁⠀⠀⢈⡜⠁⠁⣠⣭⣄⠐⢈⢦⠀⠀⢠⠒⠈⠉⣩⠉⠲⢄⢷⡀⠀
# ⠀⡴⡻⢛⣩⠥⣄⡙⢆⢀⠄⠛⠿⠋⠀⠅⡼⠁⠲⣇⠁⠀⠺⣿⠗⠀⢄⡧⡹⡆
# ⢸⢱⢀⡏⠀⣰⣄⡉⠀⠉⠒⠛⠓⠚⣚⡉⠀⠀⠀⢯⣐⠠⠴⠤⠄⡒⠛⢸⢰⡇
# ⢸⡄⡈⡇⠉⢻⡀⠙⢳⣦⣄⡉⠉⠁⣏⠤⠦⠄⠀⠀⡽⠇⠄⡀⢀⣿⡄⡊⢲⠇
# ⠀⠻⣌⠒⠀⠈⣿⣶⣬⣧⣀⠉⠙⡷⠶⠤⢤⣄⣘⣛⣁⣠⣤⠖⡏⢻⡇⢠⡏⠀
# ⠀⠀⠘⢧⠀⠀⣿⣿⣿⣿⣿⣿⣶⣧⣤⣀⣸⣇⣀⣸⣀⣀⣼⣤⣿⣾⣷⠘⡇⠀
# ⠀⠀⠀⠘⢧⡀⢸⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⡇⠀
# ⠀⠀⠀⠀⠈⠻⣆⠹⣌⢻⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⡇⠀
# ⠀⠀⠀⠀⠀⠀⠈⢳⡈⠻⣀⠀⠈⡟⠛⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⡇⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡈⠓⢾⣇⡀⠀⢸⡇⠀⢸⠏⠉⡏⢹⣃⣷⠃⢠⠇⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠲⢤⣌⣉⠙⠛⠓⠒⠚⠓⠚⠋⠉⠉⣀⣠⠏⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠛⠛⠛⠛⠛⠛⠛⠛⠉⠉⠀⠀⠀⠀

class Turtle(TurtleNode):
    def __init__(self, 
        heading_PID: PID, 
        max_speed: float, 
        max_turn_rate: float,
        **kwargs) -> None:
        super().__init__(**kwargs)
        self.heading_PID = heading_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

        self.start_position: Point = Point()

        self.waypoints: list[Point] = []
        self.current_waypoint: Point = Point()
        self.current_waypoint_index: int = None
        self.path_complete: bool = False

        self.astar: AStar = None
        self.astar_thread: Thread = None

        self.object_tracker: ObjectTracker = None
        self.object_tracker_thread: Thread = None

        self.initalizing: bool = True

        self.beer_can_in_pixels: Point = Point(x=3.85,y= -1.56)
        
        return None

    def path_callback(self, path: list[Point]) -> None:
        self.waypoints = [Point(x=point.x, y=point.y) for point in path]
        self.current_waypoint_index = len(self.waypoints) - 1
        self.current_waypoint = self.waypoints[self.current_waypoint_index]
        return None

    def switch_waypoint(self, position: Point):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        self.current_waypoint: Point = self.waypoints[self.current_waypoint_index]

        i = self.current_waypoint_index
        while i >= 0:
        # while i < len(self.waypoints) - 1:
            distance_to_waypoint: float = (distance_to(self.current_waypoint, position) 
                * self.map_meta_data.resolution)
            if ((not not i and distance_to_waypoint < 0.5)
                or (not i and distance_to_waypoint < 0.1)):
                self.heading_PID.prev_error = 0.0
                self.heading_PID.integral = 0.0
                if not self.path_complete:
                    self.current_waypoint_index += -1
                    self.current_waypoint = self.waypoints[self.current_waypoint_index]
                    # print(
                    #     f"Next Waypoint {self.current_waypoint_index + 1} / {len(self.waypoints)}: {self.current_waypoint}"
                    # )
                # i += 1
                i -= 1
                continue
            return

        self.path_complete = True

    def on_odom_callback(self) -> None:  # TODO: make this a callback from the amcl and odom fuse

        if self.current_waypoint_index is None:
            return

        # TODO: THIS IS ODOM POSITION, NOT A FUSE OF AMCL AND ODOM
        position_in_pixels = world_to_pixel_coordinates(
            Point(x=self.odom_position.x, y=self.odom_position.y), self.map_origin, self.map_meta_data
        )
        
        self.switch_waypoint(position_in_pixels)

        # get the desired heading
        desired_heading: float = heading_to(
            position_in_pixels,
            self.current_waypoint)

        # decide to turn left or right
        desired_heading = (
            desired_heading - 2 * pi
            if desired_heading - self.odom_yaw > pi
            else desired_heading
        )

        self.twist.angular.z = clamp(
            self.heading_PID.update(
                desired=desired_heading, actual=self.odom_yaw, dt=self.odom_dt
            ), -self.max_turn_rate, self.max_turn_rate
        )

        self.twist.linear.x = self.max_speed

        self.move()

        return
    
    def on_global_costmap_callback(self) -> None:

        if not self.astar_thread is None:
            return

        # if not self.object_tracker_thread is None:
        #     return

        if self.amcl_position == Point():
            return

        if self.initalizing:
            print(f"AMCL initialized...")
            self.initalizing = False

            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.move()

            self.beer_can_in_pixels = world_to_pixel_coordinates(
                self.beer_can_in_pixels, self.map_origin, self.map_meta_data)

        self.start_position = world_to_pixel_coordinates(
            self.amcl_position, self.map_origin, self.map_meta_data)

        threshold: int = 85  # inflation radius, occupancy grid gives 0 - 100 % probability of obstacle
        beer_can_radius: int = 10  # pixels
        step_size: float = 0.33  # meters

        # map image goes in as values from 0 - 100, comes out as 0's and 1's based on threshold percentage
        # self.object_tracker = ObjectTracker(self.map_image, threshold, 10)

        self.astar = AStar(
            start=self.start_position,
            goal=AStarPoint(
                self.beer_can_in_pixels.x, 
                self.beer_can_in_pixels.y, 
                0, 0, 
                radius=beer_can_radius),
            # map_image=self.object_tracker.map_image,  # HACK HACK HACK HACK
            map_image=self.map_image > threshold,
            step_size=1 / self.map_meta_data.resolution * step_size)

        # self.object_tracker_thread = Thread(target=self.object_tracker.run)
        # self.object_tracker_thread.start()
            
        plt.imshow(self.astar.map_image, cmap='gray')
        plt.scatter(self.astar.start.x, self.astar.start.y, c='c')
        plt.scatter(self.astar.goal.x, self.astar.goal.y, c='r')
        plt.pause(0.0001)

        # if below returns true, we would fail A* because we currently think 
        # we're in an obstacle.
        # if self.astar.map_image[int(self.astar.start.y)][int(self.astar.start.x)] == 255:  # HACK HACK HACK HACK, change in invader_astar too
        if self.astar.map_image[int(self.astar.start.y)][int(self.astar.start.x)]:
            return

        self.astar_thread = Thread(target=self.astar.run)
        self.astar_thread.start()

        # path: list[Point] = self.a_star(self.start_position, self.beer_can_in_pixels, map_image)

        # self.waypoints = path
        # self.current_waypoint_index = 0
        # self.current_waypoint = self.waypoints[self.current_waypoint_index]

        # plt.plot([p.x for p in path], [p.y for p in path], 'r')
        # plt.pause(0.0001)
        # plt.clf()

        # return None
    
    def on_amcl_pose_callback(self) -> None:
        return None

    def update(self) -> None:
        if self.initalizing:
            print(f"Initializing AMCL...", end='\r')
            self.initalizing = True
            self.twist.angular.z = self.max_turn_rate
            self.move()

        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()
            return

        if self.last_callback == self.__global_costmap_callback:
            self.on_global_costmap_callback()
            return
        
        if self.last_callback == self.__amcl_pose_callback:
            return
            self.on_amcl_pose_callback()

        return None


def main():
    rclpy.init()

    invader: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        max_speed=0.22,  # max speed
        max_turn_rate=1.4,  # matches constriant for 0.3 m radius turning at max speed
        namespace='',
        name="Invader")

    # invader.max_speed = 0.11
    # invader.max_turn_rate = 0.7

    while rclpy.ok():
        rclpy.spin_once(invader)

        invader.update()

        # if invader.object_tracker_thread is not None:
        #     invader.object_tracker_thread.join(timeout=0.0)
        #     if not invader.object_tracker_thread.is_alive():
        #         invader.object_tracker_thread = None
        
        if invader.astar_thread is not None:
            invader.astar_thread.join(timeout=0.0)
            if not invader.astar_thread.is_alive():
                invader.astar_thread = None
                invader.waypoints = invader.astar.path
                invader.current_waypoint_index = len(invader.waypoints) - 1
                invader.current_waypoint = invader.waypoints[invader.current_waypoint_index]
                plt.plot(
                    [p.x for p in invader.astar.closed_set.values()], 
                    [p.y for p in invader.astar.closed_set.values()], 'b.')
                plt.plot(
                    [p.x for p in invader.astar.open_set.values()],
                    [p.y for p in invader.astar.open_set.values()], 'g.')
                plt.plot([p.x for p in invader.waypoints], [p.y for p in invader.waypoints], 'r')
                plt.pause(0.0001)
                plt.clf()

        if invader.path_complete:
            invader.twist.linear.x = 0.0
            invader.twist.angular.z = 0.0
            invader.move()
            print("Path complete")
            break

    invader.close_logs()
    invader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()