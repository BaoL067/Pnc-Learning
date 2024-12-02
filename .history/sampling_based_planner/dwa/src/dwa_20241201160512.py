import math
import yaml
from typing import List, Tuple

config_path = "config.yaml"

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

class window:
    def __init__(self, v_min: float, v_max: float, w_min: float, w_max: float):
        self.v_min = v_min
        self.v_max = v_max
        self.w_min = w_min
        self.w_max = w_max

class DWA:
    def __init__(self, config_path: str):
        self.config = self.load_config(config_path)
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
        state.x += control.v * math.cos(state.yaw) * self.dt
        state.y += control.v * math.sin(state.yaw) * self.dt
        state.yaw += control.w * self.dt
        state.v = control.v
        state.w = control.w
        return state

    def trajectory_predict(self, state: State, control: Control) -> List[State]:
        trajectory = [state]
        time = 0.0
        while time <= self.predict_time:
            state = self.kinematics_model(state, control)
            trajectory.append(state)
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
        
def main():
    dwa = DWA(config_path)
    state = State(0.0, 0.0, 0.0, 0.0, 0.0)
    goal = Point(10.0, 14.0)
    obs = [Point(5.0, 5.0)]
    control, trajectory, trajectories = dwa.trajectory_evaluation(state, goal, obs)
    print(control.v, control.w)