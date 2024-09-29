import math

import matplotlib.pyplot as plt

import cv2
import numpy as np
import pathlib
import heapq

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

    def search(self, start_x, start_y, goal_x, goal_y):
        """
        input:
            start_x: start x position
            start_y: start y position
            goal_x: goal x position
            goal_y: goal y position

        output:
            path_x: x list of the final path
            path_y: y list of the final path
        """

        # construct start and goal node
        start_node = self.Node(*self.convert_coord_to_idx(start_x, start_y), 0.0, -1)
        goal_node = self.Node(*self.convert_coord_to_idx(goal_x, goal_y), 0.0, -1)

        # TODO: create open_set and closed_set
        open_set = []
        close_set = {}
        
        # add start node to open set
        heapq.heappush(open_set, (start_node.cost, start_node))

        # astar algorithm main loop
        while open_set:
            # TODO: 1. pop the node with the lowest value of the f function from the open set, and add it to the close set
            _, current_node = heapq.heappop(open_set)


    def backtracking(self, goal_node, closed_set):
        goal_x, goal_y = self.
            

    def cal_heuristic_func(self, cur_node, next_node):
        # TODO: implement the heuristic function to estimate the cost between current node and next node
        heuristic = math.hypot(cur_node.x_idx - next_node.x_idx, cur_node.y_idx - next_node.y_idx)
        return heuristic

    def convert_idx_to_coord(self, x_idx, y_idx):
        x_coord = x_idx * self.resolution + self.min_x
        y_coord = y_idx * self.resolution + self.min_y
        return x_coord, y_coord

    def convert_coord_to_idx(self, x_coord, y_coord):
        x_idx = round((x_coord - self.min_x) / self.resolution)
        y_idx = round((y_coord - self.min_y) / self.resolution)
        return x_idx, y_idx

    def get_vec_index(self, node):
        return (node.y_idx - self.min_y) * self.x_width + (node.x_idx - self.min_x)

    def check_node_validity(self, node):
        x_coord, y_coord = self.convert_idx_to_coord(node.x_idx, node.y_idx)
        # check if the current node exceeds the map range
        if x_coord < self.min_x or x_coord > self.max_x \
            or y_coord < self.min_y or y_coord > self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x_idx][node.y_idx]:
            return False

        return True

    def get_obstacle_map(self, obs_list_x, obs_list_y):
        print("Grid Map Info: ")
        self.min_x = round(min(obs_list_x))
        self.min_y = round(min(obs_list_y))
        self.max_x = round(max(obs_list_x))
        self.max_y = round(max(obs_list_y))
        