from __future__ import annotations

# python imports
import json
import matplotlib.pyplot as plt
import os
import random

# path finders imports
from .AStar import AStar
from .Dijkstra import Dijkstra
from .RRT import RRT

# support imports
from .Colors import Colors
from .Grid import Grid
from .Node import Node
from .Obstacle import Obstacle

class Scenario:
    def loader(self, filename: str) -> Scenario:
        """
        Scenario loader loads a json file using json objects to decribe a scenario.
        Below are the mandatory keys and expected value data types.

        '|' represents 'or' -- count | file means count or file

        Optional[key] means there's a default value if the key is not provided.

        ### Mandatory Keys

        bot_radius: float
        grid: {

            min_x: float
            max_x: float
            min_y: float
            max_y: float
            grid_spacing: float
        }
        obstacles: {

            radius: float | "random(min: float, max: float)"
            count | file: int | str (relative to current directory or absolute path)
        }
        start: float | "random"
        goal: float | "random"
        algorithm: {

            type: "AStar" | "Dijkstra" | "RRT"

            params: {

                --RRT
                step_length: float
                Optional[sub_step_length]: float (default=step_length / 5)
            }
        }
        """

        # cwd: str = os.path.dirname(os.path.realpath(__file__))
        # with open(os.path.join(cwd, filename), "r") as f:
        #     data = json.load(f)

        with open(filename, "r") as f:
            data = json.load(f)

        # go down the mandatory keys and set the values
        # throws an uncaught error if we are missing any keys in the scenario...

        key = "bot_radius"
        self.bot_radius = data[key]

        key = "grid"
        self.grid: Grid = Grid(
            min_x=data[key]["min_x"],
            max_x=data[key]["max_x"],
            min_y=data[key]["min_y"],
            max_y=data[key]["max_y"],
            grid_spacing=data[key]["grid_spacing"],
            obstacles=[]
        )

        key = "obstacles"
        self.has_random_obstacles = False
        self.obstacle_radius = data[key]["radius"]

        if "file" in data[key]:
            self.obstacles = Obstacle.obstacles_from_file(
                filename=data[key]["file"], 
                radius=float(self.obstacle_radius)
            )

        else:
            self.has_random_obstacles = True
            self.obstacles = Obstacle.generate_obstacles(
                count=data[key]["count"],
                radius=self.obstacle_radius,
                min_x=self.grid.min_x,
                max_x=self.grid.max_x,
                min_y=self.grid.min_y,
                max_y=self.grid.max_y
            )
            for obstacle in list(self.obstacles.values()):
                del self.obstacles[obstacle.id]
                obstacle: Node = self.grid.snap_node_to_grid(obstacle)
                self.obstacles[obstacle.id] = obstacle

        self.grid.obstacles = self.obstacles
        self.grid.inflate_obstacles(self.bot_radius)
        self.grid.inflate_bounds(self.bot_radius)
        self.grid.set_nodes()

        key = "start"
        self.has_random_start = False
        if data[key] == "random":
            self.has_random_start = True
            self.start = random.choice(list(self.grid._valid_nodes.values()))

        else:
            self.start: Node = Node(
                x=float(data[key]["x"]),
                y=float(data[key]["y"])
            )

        key = "goal"
        self.has_random_goal = False
        if data[key] == "random":
            self.has_random_goal = True
            self.goal = random.choice(list(self.grid._valid_nodes.values()))

        else:
            self.goal: Node = Node(
                x=float(data[key]["x"]),
                y=float(data[key]["y"])
            )

        key = "algorithm"
        self.algorithm: AStar | Dijkstra | RRT  = eval(data[key]["type"])(
            start=self.start,
            goal=self.goal,
            grid=self.grid,
            **data["algorithm"]["params"],
        )

        return self

    def plot_start_and_goal(self, ax) -> None:
        ax.plot(self.start.x, self.start.y, "go", markersize=8)
        ax.plot(self.goal.x, self.goal.y, "ro", markersize=8)

    def plot_obstacles(self, ax: plt.Axes, color: str):
        """
        Plots the obstacles in the grid

        :param ax: axis to plot on
        """
        for _id, obstacle in self.obstacles.items():
            ax.add_artist(plt.Circle(
                (obstacle.x, obstacle.y),
                obstacle.radius,
                color=color,
            ))

    def plot_nodes(self, ax: plt.Axes, invalid_nodes=True, valid_nodes=True):
        """
        Plots the nodes in the grid

        :param ax: axis to plot on
        """
        if invalid_nodes:
            for node in self.grid._invalid_nodes.values():
                ax.plot(
                    node.x, node.y, 
                    color=(
                        Colors.dark_red 
                        if self.grid.node_in_obstacle(node) 
                        else Colors.light_grey
                    ), 
                    marker="."
                )

        if valid_nodes:
            for node in self.grid._valid_nodes.values():
                ax.plot(node.x, node.y, color=Colors.light_grey, marker=".")

    def plot_path(self, ax: plt.Axes, color: str) -> None:
        """
        Plots the path on the given axes
        """
        ax.plot(
            [node.x for node in self.algorithm._path],
            [node.y for node in self.algorithm._path],
            color=color,
            linewidth=3
        )

    def plot_open_set(self, ax: plt.Axes, color: str) -> None:
        """
        Plots the open set on the given axes
        """
        for node in self.algorithm._open_set.values():
            ax.text(
                node.x, node.y, f"{node.total_cost:.1f}", 
                color=color, ha="center", va="center",
                fontsize=6
            )

    def plot_closed_set(self, ax: plt.Axes, color: str) -> None:
        """
        Plots the closed set on the given axes
        """
        for node in self.algorithm._closed_set.values():
            ax.text(
                node.x, node.y, f"{node.total_cost:.1f}", 
                color=color, ha="center", va="center",
                fontsize=6
            )