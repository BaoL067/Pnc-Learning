import math
import matplotlib.pyplot as plt

def plot_robot(x, y, radius):
    """
    Plot the robot as a circle.
    :param x: x-coordinate of the robot's center
    :param y: y-coordinate of the robot's center
    :param radius: Radius of the robot
    """
    theta = [i * 0.1 for i in range(int(2 * math.pi / 0.1) + 1)]
    points_x = [x + radius * math.cos(t) for t in theta]
    points_y = [y + radius * math.sin(t) for t in theta]
    plt.plot(points_x, points_y, "-m")

def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    """
    Plot an arrow to represent the robot's orientation.
    :param x: x-coordinate of the arrow's starting point
    :param y: y-coordinate of the arrow's starting point
    :param yaw: Robot's orientation (radians)
    :param length: Length of the arrow
    :param width: Width of the arrowhead
    """
    dx = length * math.cos(yaw)
    dy = length * math.sin(yaw)
    plt.arrow(x, y, dx, dy, head_width=width, color="blue")