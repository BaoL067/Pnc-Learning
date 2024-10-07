import math 
import random

import sys
import matplotlib.pyplot as plt
import cv2
import pathlib

SHOW_SAMPLING_PROCESS = True

class RRTStar:

    class Node:
        def __init__(self, x, y):
            # the position of node
            self.x = x
            self.y = y
            # the path between current node and its parent
            self.path_x = []
            self.path_y = []
            # the cost of node
            self.cost = 0
            # the parent of node
            self.parent = None

        def __init__(self,
                    start,
                    goal,
                    x_rand_area,
                    y_rand_area,
                    obstacle_map = None,
                    extend_dist = 10.0,
                    path_resolution = 1.0,
                    goal_sample_rate = 10,
                    max_iter = 1000,
                    near_nodes_dist_threshold = 30.0,
                    search_until_max_iter = False):

            self.start = self.Node(start[0], start[1])
            self.end = self.Node(goal[0], goal[1])
            self.x_min_rand = x_rand_area[0]
            self.x_max_rand = x_rand_area[1]
            self.y_min_rand = y_rand_area[0]
            self.y_max_rand = y_rand_area[1]
            self.obstacle_map = obstacle_map
            self.extend_dist = extend_dist
            self.path_resolution = path_resolution
            self.goal_sample_rate = goal_sample_rate
            self.max_iter = max_iter

            self.near_nodes_dist_threshold = near_nodes_dist_threshold
            self.search_until_max_iter = search_until_max_iter

            self.goal_node = self.Node(goal[0], goal[1])
            self.node_list = []

        def planning(self):
            self.draw_start_and_goal()
            self.node_list = [self.start]
            for i in range(self.max_iter):
                # TODO: 1. generate random node within the x/y range


                # TODO: 2. find the nearest node to random node in the tree


                # TODO: 3. Steer the nearest_node to random with extend_dist


                if self.check_collision_with_obs_map(new_node, self.obstacle_map):
                    # TODO: 4. find nodes near the new_node, and choose the parent which maintains a minimum-cost from the start node


                    # TODO: 5. rewrite the tree and draw current node and the edge 

        