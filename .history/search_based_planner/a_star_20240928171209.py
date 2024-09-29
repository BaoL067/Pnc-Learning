import math

import matplotlib.pyplot as plt

import cv2
import numpy as np
import pathlib

show_animation = True

class AStarPlanner:
    def __init__(self, obs_list_x, obs_list_y, resolution, min_safety_dist):
        """
        obs_list_x: x list of obstacles
        obs_list_y: y list of obstacles
        resolution: grid map resolution
        min_safety_dist: minimum safety distance to obstacle
        """
        self.resolution = resolution
        self.min_safety_dist = min_safety_dist
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.x_width, self.y_width = 0, 0
        # create 2d grid map
        self.obstacle_map = None
        self.get_obstacle_map(obs_list_x, obs_list_y)
        # create motion model
        self.motion_model = self.get_motion_model()

    class Node:
        def __init__(self, x_idx, y_idx, cost, parent_idx):
            self.x_idx = x_idx  # index of grid map
            self.y_idx = y_idx  # index of grid map
            self.cost = cost # g value
            self.parent_idx = parent_idx

        def __str__(self):
            return str(self.x_idx) + "," + str(self.y_idx) + "," + str(
                self.cost) + "," + str(self.parent_idx)

    def search(self, start_x, start_y, goal_x, goal_y)