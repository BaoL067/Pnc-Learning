import math
import yaml
import copy
import pathlib
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt

config_path = str(pathlib.Path.cwd()) + r"\sampling_based_planner\dwa\config\config.yml"

class State:
    def __init__(self, x: float, y: float, yaw: float, v: float, w: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

class Control:
    def __init__(self, v: float, w: float):
        self.v = v
        self.w = w

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

class Window:
    def __init__(self, v_min: float, v_max: float, w_min: float, w_max: float):
        self.v_min = v_min
        self.v_max = v_max
        self.w_min = w_min
        self.w_max = w_max

class DWA:
    def __init__(self, config_path: str):
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        self.dt = config["dt"]
        self.v_min = config["vMin"]
        self.v_max = config["vMax"]
        self.w_min = config["wMin"]
        self.w_max = config["wMax"]
        self.predict_time = config["predictTime"]
        self.a_v_max = config["aVmax"]
        self.a_w_max = config["aWmax"]
        self.v_resolution = config["vSample"]
        self.w_resolution = config["wSample"]
        self.alpha = config["alpha"]
        self.beta = config["beta"]
        self.gamma = config["gamma"]
        self.radius = config["radius"]
        self.judge_distance = config["judgeDistance"]

    def kinematics_model(self, state: State, control: Control) -> State:
        new_state = State(
            x=state.x + control.v * math.cos(state.yaw) * self.dt,
            y=state.y + control.v * math.sin(state.yaw) * self.dt,
            yaw=state.yaw + control.w * self.dt,
            v=control.v,
            w=control.w
        )
        return new_state

    def trajectory_predict(self, state: State, control: Control) -> List[State]:
        trajectory = [copy.deepcopy(state)]
        time = 0.0
        while time <= self.predict_time:
            new_state = copy.deepcopy(trajectory[-1])
            new_state = self.kinematics_model(new_state, control)
            trajectory.append(new_state)
            time += self.dt
        return trajectory

    def get_obs_dis(self, state: State, obs: List[Point]) -> float:
        min_dis = float('inf')
        for o in obs:
            dis = math.hypot(state.x - o.x, state.y - o.y)
            if dis < min_dis:
                min_dis = dis
        return min_dis

    def dynamic_window(self, state: State, obs: List[Point]) -> Window:
        acc_limit = Window(
            state.v - self.a_v_max * self.dt,
            state.v + self.a_v_max * self.dt,
            state.w - self.a_w_max * self.dt,
            state.w + self.a_w_max * self.dt
        )
        obs_limit = Window(
            self.v_min,
            math.sqrt(2 * self.get_obs_dis(state, obs) * self.a_v_max),
            self.w_min,
            self.w_max
        )
        return Window(
            max(self.v_min, obs_limit.v_min, acc_limit.v_min),
            min(self.v_max, obs_limit.v_max, acc_limit.v_max),
            max(self.w_min, obs_limit.w_min, acc_limit.w_min),
            min(self.w_max, obs_limit.w_max, acc_limit.w_max)
        )

    def _obs(self, trajectory: List[State], obs: List[Point]) -> float:
        min_dis = float('inf')
        for o in obs:
            for t in trajectory:
                dis = math.hypot(t.x - o.x, t.y - o.y)
                if dis < min_dis:
                    min_dis = dis
        return min_dis if min_dis < self.radius + 0.2 else self.judge_distance

    def _vel(self, trajectory: List[State]) -> float:
        return trajectory[-1].v

    def trajectory_evaluation(
        self, state: State, goal: Point, obs: List[Point]
    ) -> Tuple[Control, List[State], List[List[State]]]:
        best_score = -float('inf')
        optimal_trajectory = []
        optimal_control = None
        dynamic_window = self.dynamic_window(state, obs)
        trajectories = []

        v = dynamic_window.v_min
        while v <= dynamic_window.v_max:
            w = dynamic_window.w_min
            while w <= dynamic_window.w_max:
                control = Control(v, w)
                trajectory = self.trajectory_predict(state, control)
                trajectories.append(trajectory)

                obs_score = self.beta * self._obs(trajectory, obs)
                vel_score = self.gamma * self._vel(trajectory)
                heading_score = 1 / math.hypot(
                    trajectory[-1].x - goal.x, trajectory[-1].y - goal.y
                )
                total_score = obs_score + vel_score + heading_score

                if total_score > best_score:
                    best_score = total_score
                    optimal_trajectory = trajectory
                    optimal_control = control
                w += self.w_resolution
            v += self.v_resolution

        return optimal_control, optimal_trajectory, trajectories

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

def main():
    dwa = DWA(config_path)
    state = State(0.0, 0.0, 0.0, 0.0, 0.0)
    goal = Point(10.0, 14.0)
    obs = [Point(*p) for p in [
        (-1, -1), (0, 2), (4, 2), (5, 4), (5, 5), (5, 6), (5, 9), (8, 9),
        (7, 9), (8, 10), (9, 11), (12, 13), (12, 12), (15, 15), (13, 13),
        (5, 10), (3, 5), (4, 4), (4, 11), (15, 5), (8, 13), (5, 13),
        (5, 5), (2, 8)
    ]]
    trajectory = []  # Optimal trajectory
    x_, y_ = [], []

    # Add a flag for ESC key detection
    def on_key(event):
        if event.key == "escape":  # ESC key
            print("ESC key pressed. Exiting simulation...")
            sys.exit()  # Exit the program immediately

    # Connect the key press event
    fig = plt.figure()
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Connect the key press event
    fig = plt.figure()
    fig.canvas.mpl_connect("key_press_event", on_key)

    while True:

        control, optimal_trajectory, all_trajectories = dwa.trajectory_evaluation(state, goal, obs)
        state = dwa.kinematics_model(state, control)
        trajectory.append(state)

        x_.append(state.x)
        y_.append(state.y)

        # Plotting
        plt.clf()
        plt.plot([state.x], [state.y], "xr")  # Current state
        plt.plot([goal.x], [goal.y], "rs")    # Goal point

        plot_obstacles(obs)                   # Plot obstacles
        plot_arrow(state.x, state.y, state.yaw)  # Plot robot's orientation
        plot_robot(state.x, state.y, dwa.radius) # Plot robot
        plot_trajectories(all_trajectories, optimal_trajectory)  # Plot trajectories

        # Plot robot's actual path
        plt.plot(x_, y_, "r")
        plt.pause(0.01)

        # Termination condition
        if math.hypot(state.x - goal.x, state.y - goal.y) <= dwa.radius:
            print("Robot has reached the goal.")
            break

    # Save result
    plt.savefig("dwa_demo.png")
    plt.show()

if __name__ == "__main__":
    main()