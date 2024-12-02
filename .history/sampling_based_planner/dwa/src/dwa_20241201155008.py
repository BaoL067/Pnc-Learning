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
        