import math

import matplotlib.pyplot as plt

import cv2
import numpy as np
import pathlib
import heapq

show_animation = True
img_path = str(pathlib.Path.cwd()) + "\search_based_planner\maps\map1.png"

class AStarPlanner:
    def __init__(self, obstacle_x_list, obstacle_y_list , resolution, min_safety_dist):
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
        self.get_obstacle_map(obstacle_x_list, obstacle_y_list)
        # create motion model
        self.motion_model = self.get_motion_model()

    class Node:
        def __init__(self, x_idx, y_idx, cost, parent_idx):
            self.x_idx = x_idx  # index of grid map
            self.y_idx = y_idx  # index of grid map
            self.cost = cost    # g value
            self.parent_idx = parent_idx

        def __lt__(self, other):
            return self.cost < other.cost

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
        nodes_expanded = 0

        # astar algorithm main loop
        while open_set:
            # TODO: 1. pop the node with the lowest value of the f function from the open set, and add it to the close set
            current_cost, current_node = heapq.heappop(open_set)
            current_idx = self.get_vec_index(current_node)

            # skip if the current node is already in the closed set
            if current_idx in close_set:
                continue

            # add the current node to the closed set
            close_set[current_idx] = current_node
            nodes_expanded += 1

            # plot current node
            if show_animation:
                plt.plot(*self.convert_idx_to_coord(current_node.x_idx, current_node.y_idx), marker='s', 
                    color='dodgerblue', alpha=0.2)
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(close_set) % 10 == 0:
                    plt.pause(0.0000001)

            # TODO: 2. determine whether the current node is the goal, and if so, stop searching
            if current_node.x_idx == goal_node.x_idx and current_node.y_idx == goal_node.y_idx:
                goal_node.parent_idx = current_node.parent_idx
                print("Find the goal node")
                print("Nodes expanded: ", nodes_expanded)
                break

            # TODO: 3. expand neighbors of the current node
            for i, (dx, dy, cost) in enumerate(self.motion_model):
                next_node = self.Node(current_node.x_idx + dx, current_node.y_idx + dy,
                                      current_node.cost + cost, self.get_vec_index(current_node))
                next_idx = self.get_vec_index(next_node)
                # skip if the next node is occupied or visited  
                if not self.check_node_validity(next_node) or next_idx in close_set:
                    continue
                
                next_cost = current_node.cost + cost
                if next_idx not in close_set or next_cost < close_set[self.get_vec_index(next_node)].cost:
                    next_node.cost = next_cost
                    heuristics = self.cal_heuristic_func(next_node, goal_node)
                    heapq.heappush(open_set, (next_node.cost + heuristics, next_node))
        
        if not close_set:
            print("open_set is empty, can't find path")
            return [], []
        
        # TODO: 4. backtrack to get the shortest path
        path_x, path_y = self.backtracking(goal_node, close_set)
        return path_x, path_y

    def backtracking(self, goal_node, closed_set):
        goal_x, goal_y = self.convert_idx_to_coord(goal_node.x_idx, goal_node.y_idx)
        path_x, path_y = [goal_x], [goal_y]

        # TODO: backtracking from goal node to start node to extract the whole path
        current_node = goal_node

        while True:
            pre_node = closed_set[current_node.parent_idx]
            x, y = self.convert_idx_to_coord(pre_node.x_idx, pre_node.y_idx)
            path_x.append(x)
            path_y.append(y)
            if pre_node.parent_idx == -1:
                break
            current_node = pre_node
         
        return path_x, path_y

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
        print("min_x: ", self.min_x)
        print("min_y: ", self.min_y)
        print("max_x: ", self.max_x)
        print("max_y: ", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("Map x: ",  self.x_width)
        print("Map y: ",  self.y_width)

        # initialize obstacle map
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for x_idx in range(self.x_width):
            for y_idx in range(self.y_width):
                x_coord, y_coord = self.convert_idx_to_coord(x_idx, y_idx)
                for obs_x, obs_y in zip(obs_list_x, obs_list_y):
                    dist = math.hypot(obs_x - x_coord, obs_y - y_coord)
                    if dist <= self.min_safety_dist:
                        self.obstacle_map[x_idx][y_idx] = True
                        break

    def get_motion_model(self):
        # 8-connected motion model
        motion_model = [[1, 0, 1],
                        [0, 1, 1],
                        [-1, 0, 1],
                        [0, -1, 1],
                        [-1, -1, math.sqrt(2)],
                        [-1, 1, math.sqrt(2)],
                        [1, -1, math.sqrt(2)],
                        [1, 1, math.sqrt(2)]]
        return motion_model
                        
def preprocess_image(image, threshold):
    # convert to gray image
    gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # binarization
    _, binary_img = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY)
    return binary_img

def extract_obstacle_list_from_img(binary_img):
    obstacle_indices = np.where(binary_img == 0)
    obstacle_x_list = obstacle_indices[1].tolist()
    obstacle_y_list = (binary_img.shape[0] - obstacle_indices[0] - 1).tolist()

    return obstacle_x_list, obstacle_y_list

def main():
    # start and goal position
    start_x, start_y = 20.0, 40.0   # [m]
    goal_x, goal_y = 140.0, 40.0    # [m]
    grid_res = 2.0                  # [m]
    min_safety_dist = 1.0           # [m]
    
    # read map
    map_img = cv2.imread() 
    # cv2.imshow('map_img', map_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    binary_img = preprocess_image(map_img, 127)
    obstacle_x_list, obstacle_y_list = extract_obstacle_list_from_img(binary_img)
    
    # plot map info
    if show_animation:
        plt.figure(figsize=(12, 12))
        plt.axis("equal")
        plt.plot(obstacle_x_list, obstacle_y_list, 'sk', markersize=2)
        plt.plot(start_x, start_y, marker='*', color='lime', markersize=8)
        plt.plot(goal_x, goal_y, marker='*', color='r', markersize=8)

    a_star = AStarPlanner(obstacle_x_list, obstacle_y_list, grid_res, min_safety_dist)
    path_x, path_y = a_star.search(start_x, start_y, goal_x, goal_y)
    print("Path length: ", len(path_x))
    # plot searched path
    if show_animation:
        plt.plot(path_x, path_y, ".-", color="royalblue")
        plt.show()
    
if __name__ == '__main__':
    main()