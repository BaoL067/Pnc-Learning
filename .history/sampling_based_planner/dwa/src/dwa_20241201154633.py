import math
import yaml
from typing import List, Tuple

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


        