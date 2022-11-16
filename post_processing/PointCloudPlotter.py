from __future__ import annotations

import matplotlib.pyplot as plt

if __name__ == "__main__":

    points: list[tuple[float, float]] = []

    # read file
    with open("Tester_point_cloud.csv", "r") as f:
        lines = f.readlines()

    for line in lines[1:]:
        line = line.split(",")
        points.append((float(line[0]), float(line[1])))

    fig, ax = plt.subplots()
    ax.scatter([point[0] for point in points], [point[1] for point in points])
    ax.set_aspect("equal")
    plt.show()