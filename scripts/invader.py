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
from nav_msgs.msg import MapMetaData

# personal imports
from PythonRobotics.PathPlanning.DubinsPath import dubins_path_planner as dpp
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
    
    # point_in_pixels: Point = Point(
    #     x=point.x / map_meta_data.resolution + origin_in_pixels.x,
    #     y=point.y / map_meta_data.resolution + origin_in_pixels.y)
        
    
    return point_in_pixels

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
        self.parent_polygon_index: int = None

        self.waypoints: list[Point] = []
        self.current_waypoint: Point = Point()
        self.current_waypoint_index: int = None
        self.path_complete: bool = False

        self.prev_map = None
        
        return None

    def switch_waypoint(self, position: Point):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        self.current_waypoint: Point = self.waypoints[self.current_waypoint_index]

        i = self.current_waypoint_index
        while i < len(self.waypoints) - 1:
            distance_to_waypoint: float = (distance_to(self.current_waypoint, position) 
                * self.map_meta_data.resolution)
            if distance_to_waypoint < 0.5:

                self.heading_PID.prev_error = 0.0
                self.heading_PID.integral = 0.0
                if not self.path_complete:
                    self.current_waypoint_index += 1
                    self.current_waypoint = self.waypoints[self.current_waypoint_index]
                    # print(
                    #     f"Next Waypoint {self.current_waypoint_index + 1} / {len(self.waypoints)}: {self.current_waypoint}"
                    # )
                i += 1
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

        if not not self.waypoints:
            return
            
        threshold = 75
        plt.clf()

        if self.prev_map is not None:
            # diff = np.abs(self.prev_map ^ (self.map_image > threshold))
            # plt.imshow(diff, cmap='gray')
            plt.imshow(self.map_image)

        self.prev_map = self.map_image > threshold

        plt.imshow(self.map_image > threshold, cmap='gray', )
        
        point_in_pixels = world_to_pixel_coordinates(
            Point(x=self.amcl_position.x, y=self.amcl_position.y), self.map_origin, self.map_meta_data
        )
        plt.scatter(point_in_pixels.x, point_in_pixels.y, c='r', marker='x')

        if self.start_position == Point():
            self.start_position = point_in_pixels

        img = np.array(self.map_image, dtype=np.uint8)
        #make 3 channels
        img = np.stack((img, img, img), axis=2)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(img, threshold, 255, 0)

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 2)

        polygons = []
        for contour in contours:
            poly = []
            for point in contour:
                poly.append(vg.Point(point[0][0], point[0][1]))
            polygons.append(poly)

        if self.start_position != Point():
            self.parent_polygon_index = -1

            g = vg.VisGraph()
            try:
                g.build(polygons, workers=4)
            except ZeroDivisionError:
                return
            except UnboundLocalError:
                return

            for polygon in g.graph.polygons:
                if polygon_crossing(vg.Point(self.start_position.x, self.start_position.y), g.graph.polygons[polygon]):
                    if polygon > self.parent_polygon_index:
                        print(polygon)
                        self.parent_polygon_index = polygon
                        
            for i in range(len(polygons) - 1, -1, -1):
                if i != self.parent_polygon_index:
                    if hierarchy[0][i][3] > self.parent_polygon_index + 1:
                        del polygons[i]
                
            g = vg.VisGraph()
            g.build(polygons, workers=4)

            # plot contours
            for poly in polygons:
                plt.plot([p.x for p in poly], [p.y for p in poly], c='r')

                plt.plot([poly[-1].x, poly[0].x], [poly[-1].y, poly[0].y], c='r')

            beer_can_in_pixels = world_to_pixel_coordinates(
                Point(x=3.85,y= -1.56), self.map_origin, self.map_meta_data
            )
            # use visgraph to find a path
            shortest: list[vg.Point] = g.shortest_path(
                vg.Point(
                    point_in_pixels.x, 
                    point_in_pixels.y),
                vg.Point(
                    beer_can_in_pixels.x,
                    beer_can_in_pixels.y)
            )

            # shortest.insert(2, vg.Point(6.5, 6.5))
            # shortest.insert(4, vg.Point(8.5, 9.5))
            plt.plot(
                [point.x for point in shortest],
                [point.y for point in shortest],
                color="#ffff00", linewidth=2)# connect the points with dubins curves

            path_x = []
            path_y = []

            for i, point in enumerate(shortest[:-1]):
                start = point
                end = shortest[i + 1]
                start_yaw = (yaw_from_points(start, end) 
                    if i > 0 
                    else self.odom_yaw)
                end_yaw = (yaw_from_points(end, shortest[i + 2]) 
                    if i < len(shortest) - 2 
                    else np.deg2rad(0))
                radius = (1 / 0.05) * 0.33
                curvature = 1 / radius
                step_size = 1 * 0.05
                edge_x, edge_y, path_yaw, mode, _ = dpp.plan_dubins_path(
                    start.x,
                    start.y,
                    start_yaw,
                    end.x,
                    end.y,
                    end_yaw,
                    curvature,
                    step_size)

                path_x.extend(edge_x)
                path_y.extend(edge_y)

                # highlighting points that are invliad
                # for x, y in zip(path_x, path_y):
                #     p = vg.Point(x, y)
                #     if g.point_in_polygon(p) != -1:
                #         plt.plot(x, y, "o", color=Colors.light_purple)
            

            # self.waypoints: list[vg_Point] = [p for p in shortest]
            self.waypoints = []

            for x, y in zip(path_x, path_y):
                self.waypoints.append(Point(x=x, y=y, z=0.0))
            
            self.current_waypoint_index = 0
            self.current_waypoint = self.waypoints[self.current_waypoint_index]

            plt.plot(path_x, path_y, color=Colors.light_blue, marker="o", markersize=1)

            plt.pause(0.001)
        return None
    
    def on_amcl_pose_callback(self) -> None:
        return None

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()
            return

        if self.last_callback == self.__global_costmap_callback:
            self.on_global_costmap_callback()
            return
        
        if self.last_callback == self.__amcl_pose_callback:
            self.on_amcl_pose_callback()
            return

        return None


def main():
    rclpy.init()

    invader: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
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

        if invader.path_complete:
            print("Path complete")
            break

    invader.close_logs()
    invader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()