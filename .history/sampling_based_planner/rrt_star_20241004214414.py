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
            self.end = self.Node()
        