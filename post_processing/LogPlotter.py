from __future__ import annotations

import math
import matplotlib.pyplot as plt
import re
import os.path

from Colors import Colors

def read_log_file(filename: str) -> list[str]:
    with open(filename, 'r') as f:
        lines = f.readlines()
    return lines

def main() -> None:
    turtle_name = "Tester"
    

    dir =  f"{os.path.expanduser('~')}/.ros/log/sgaar"
    heading_log: list[str] = read_log_file(f"{dir}/{turtle_name}_heading_log.csv")
    command_log: list[str] = read_log_file(f"{dir}/{turtle_name}_command_log.csv")
    pose_log: list[str] = read_log_file(f"{dir}/{turtle_name}_pose_log.csv")
    # waypoint_log: list[str] = read_log_file(f"/home/thomas/.ros/log/seagraves_unmanned_systems_pkg/{turtle_name}waypoint_log.csv")

    # create two sub plots
    # one plot is an x y position graph where the color of the line is a funciton of the time
    # the other plot is a time vs. velocity graph

    current_time: list[float] = []
    current_x: list[float] = []
    current_y: list[float] = []
    current_z: list[float] = []
    current_roll: list[float] = []
    current_pitch: list[float] = []
    current_yaw: list[float] = []
    
    command_time: list[float] = []
    command_x: list[float] = []   # forward
    command_yaw: list[float] = []  # z in the angular vector

    heading_time: list[float] = []
    heading_desired: list[float] = []
    heading_actual: list[float] = []

    waypoint_time: list[float] = []
    waypoint_x: list[float] = []
    waypoint_y: list[float] = []
    waypoint_z: list[float] = []

    waypoint_number_time: list[float] = []
    waypoint_number: list[int] = []

    start_time: int = None

    # time,position_x,position_y,position_z,roll,pitch,yaw
    for line in pose_log[1:]:
        line = line.split(",")

        current_time.append(float(line[0]))
        if start_time is None:
            start_time = int(current_time[-1])
            current_time[-1] = 0
        else:
            current_time[-1] -= start_time
        current_x.append(float(line[1]))
        current_y.append(float(line[2]))
        current_z.append(float(line[3]))
        current_roll.append(float(line[4]))
        current_pitch.append(float(line[5]))
        current_yaw.append(float(line[6]))
        
    # time,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z
    for line in command_log[1:]:
        line = line.split(",")

        line_time = float(line[0])
        if command_time:
            command_time.append(line_time - start_time)
            command_x.append(command_x[-1])
            command_yaw.append(command_yaw[-1])

        command_time.append(float(line[0]) - start_time)
        command_x.append(float(line[1]))
        command_yaw.append(float(line[6]))

    # time,desired_heading,actual_heading
    for line in heading_log[1:]:
        line = line.split(",")
        
        heading_time.append(float(line[0]) - start_time)
        heading_desired.append(float(line[1]))
        heading_actual.append(float(line[2]))

    # time,waypoint_x,waypoint_y,waypoint_z
    # for i, line in enumerate(waypoint_log[1:]):
    #     line = line.split(",")

    #     if waypoint_time:
    #         waypoint_number_time.append(float(line[0]) - start_time)
    #         waypoint_number.append(waypoint_number[-1])            

    #     waypoint_time.append(float(line[0]) - start_time)
    #     waypoint_x.append(float(line[1]))
    #     waypoint_y.append(float(line[2]))
    #     waypoint_z.append(float(line[3]))

    #     waypoint_number_time.append(float(line[0]) - start_time)
    #     waypoint_number.append(i)

    fig = plt.figure()
    plt.style.use('dark_background')
    plt.set_cmap("Blues")
    position_subplot = plt.subplot2grid((3, 2), (0, 0), colspan=1)
    rotation_subplot = plt.subplot2grid((3, 2), (1, 0), colspan=1)
    command_subplot = plt.subplot2grid((3, 2), (2, 0), colspan=1)
    # waypoint_subplot = plt.subplot2grid((3, 2), (3, 0), colspan=1)
    xy_subplot = plt.subplot2grid((3, 2), (0, 1), rowspan=3)

    twin_position = position_subplot.twinx()
    twin_command = command_subplot.twinx()

    fig.set_size_inches(9, 6)

    fig.set_facecolor(Colors.dark_grey)
    for subplot in [
        position_subplot, 
        twin_position,
        rotation_subplot, 
        command_subplot, 
        twin_command,
        # waypoint_subplot,
        xy_subplot,
    ]:
        subplot.set_facecolor(Colors.grey)
        subplot.title.set_color(Colors.grey)
        subplot.xaxis.label.set_color(Colors.grey)
        subplot.yaxis.label.set_color(Colors.grey)
        subplot.tick_params(colors=Colors.grey)
    
    position_subplot.set_title("Position")
    rotation_subplot.set_title("Rotation")
    command_subplot.set_title("Commands")
    # waypoint_subplot.set_title("Waypoints")
    xy_subplot.set_title("XY Position")

    min_position_axis = min(min(current_x), min(current_y))
    max_position_axis = max(max(current_x), max(current_y))
    min_position_axis = min_position_axis - (max_position_axis - min_position_axis) / 10
    max_position_axis = (max_position_axis - min_position_axis) / 10 + max_position_axis

    # plotting
    # position subplot
    # plotting vertical lines for each waypoint
    for i, time in enumerate(waypoint_number_time):
        # position_subplot.text(time, middle_x, str(waypoint_number[i]), color=Colors.white)
        position_subplot.axvline(time, color=Colors.light_grey, linestyle="dashed", alpha=0.2)

    position_subplot.plot(current_time, current_x, color=Colors.red, label="x")
    twin_position.plot(current_time, current_y, color=Colors.blue, label="y")
    position_subplot.plot(waypoint_time, waypoint_x, color=Colors.red, linestyle="dashed", label="waypoint x")
    twin_position.plot(waypoint_time, waypoint_y, color=Colors.blue, linestyle="dashed", label="waypoint y")

    position_subplot.set_ylim(min_position_axis, max_position_axis)
    twin_position.set_ylim(min_position_axis, max_position_axis)
    xy_subplot.set_aspect('equal')

    # rotation subplot
    rotation_subplot.plot(heading_time, heading_actual, color=Colors.red, label="actual heading dot")
    rotation_subplot.plot(heading_time, heading_desired, color=Colors.blue, label="desired heading dot")

    # command subplot
    command_subplot.plot(command_time, command_x, color=Colors.red, label="x", marker="o", markersize=2)
    twin_command.plot(command_time, command_yaw, color=Colors.blue, label="yaw", marker="o", markersize=2)

    # waypoint_subplot.plot(waypoint_number_time, waypoint_number, color=Colors.red, label="waypoint number")

    # xy subplot
    xy_subplot.scatter(current_x, current_y, c=current_time, label="xy", marker="o", s=4)
    xy_subplot.plot(waypoint_x, waypoint_y, color=Colors.red, label="waypoints", marker="o", markersize=4)
    # xy_subplot.plot(waypoint_x[0], waypoint_y[0], color=Colors.green, label="start", marker="o", markersize=4)
    # xy_subplot.plot(waypoint_x[-1], waypoint_y[-1], color=Colors.white, label="end", marker="o", markersize=4)

    # for i, waypoint in enumerate(zip(waypoint_x, waypoint_y)):
    #     xy_subplot.text(waypoint[0], waypoint[1], f"{i}", color=Colors.white, ha="center", va="center")

    xy_subplot.set_xlim(min_position_axis, max_position_axis)
    xy_subplot.set_ylim(min_position_axis, max_position_axis)
    xy_subplot.set_aspect('equal')
    
    # labels
    position_subplot.set_ylabel('x (m)')
    twin_position.set_ylabel('y (m)', rotation=270, labelpad=15)

    rotation_subplot.set_ylabel('yaw (deg)')

    command_subplot.set_ylabel('x (m/s)')
    twin_command.set_ylabel('yaw (rad/s)', rotation=270, labelpad=15)
    command_subplot.set_xlabel('time (s)')

    # waypoint_subplot.set_xlabel('time (s)')

    xy_subplot.set_xlabel('x (m)')
    xy_subplot.set_ylabel('y (m)')

    # legends
    position_subplot.legend(loc='upper left')
    twin_position.legend(loc='lower right')
    rotation_subplot.legend(loc='upper left')
    command_subplot.legend(loc='upper left')
    twin_command.legend(loc='upper right')
    # waypoint_subplot.legend(loc='upper left')
    xy_subplot.legend()


    # plot colorbar for xy subplot
    cbar = plt.colorbar(xy_subplot.collections[0], ax=xy_subplot)
    cbar.set_label('time (s)')
    cbar.ax.yaxis.label.set_color(Colors.grey)
    cbar.ax.tick_params(colors=Colors.grey)

    fig.set_tight_layout(True)
    plt.show()
    fig.savefig("telemetry.png", facecolor=fig.get_facecolor(), edgecolor='none')

if __name__ == "__main__":
    main()
