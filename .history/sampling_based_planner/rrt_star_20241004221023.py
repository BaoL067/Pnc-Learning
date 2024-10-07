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


                    # TODO: 5. rewrite the tree and draw current node and the edge between it and its parent node

                # TODO: 6. check if the tree has extended to the vicinity of the goal point, if so, select a suitable
                # node directly to connect to the goal, if the edge does not collide with obstacles, the search can
                # be finished, return the entire path which is extracted by backtracking.
                if ((not self.search_until_max_iter) and new_node):

            
            print("reach max iteration")
            # TODO: 7. reach max iteration, backtracking to extract the path between the last node and the start node

            return None

        def backtracking(self, goal_idx):
            """
                backtracking from last index to retrieve the entire path.
            """
            path = [[self.end.x, self.end.y]]
            node = self.node_list[goal_idx]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])

            return path

        def check_collision_with_obs_map(self, node, obstacle_map):
            if node is None:
                return False

            for x, y in zip(node.path_x, node.path_y):
                if (obstacle_map[round(x)][round(y)] == True ):
                    return False  # collision

            return True  # safe

        def draw_start_and_goal(self):
            plt.plot(self.start.x, self.start.y, marker='*', color='darkviolet', markersize=8, zorder=100000)
            plt.plot(self.end.x, self.end.y, marker='*', color='r', markersize=8, zorder=100000)

        def draw_node_and_edge(self, node):
            # for stopping simulation with the esc key.
            plg.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'escape' else None])  
            plt.plot(node.x, node.y, '.', color='cornflowerblue')
            if node.parent:
                plt.plot(node.path_x, node)   