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

def plot_trajectories(trajectories, optimal_trajectory=None):
    """
    Plot sampled trajectories and the optimal trajectory.
    :param trajectories: All sampled trajectories (List of List of States)
    :param optimal_trajectory: Optimal trajectory (List of States)
    """
    # Plot sampled trajectories
    for traj in trajectories:
        predict_x = [s.x for s in traj]
        predict_y = [s.y for s in traj]
        plt.plot(predict_x, predict_y, "-y")  # Yellow for sampled trajectories

    # Plot the optimal trajectory
    if optimal_trajectory:
        predict_x = [s.x for s in optimal_trajectory]
        predict_y = [s.y for s in optimal_trajectory]
        plt.plot(predict_x, predict_y, "-g")  # Green for the optimal trajectory

def plot_obstacles(obstacles):
    """
    Plot obstacles on the map.
    :param obstacles: List of obstacle points (List of Point)
    """
    for obs in obstacles:
        plt.plot([obs.x], [obs.y], "ok")  # Black dots for obstacles

def save_figure(filename):
    """
    Save the current plot to a file.
    :param filename: Name of the file to save the plot
    """
    plt.savefig(filename)