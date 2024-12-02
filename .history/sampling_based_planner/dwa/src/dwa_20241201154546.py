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

class DWA: