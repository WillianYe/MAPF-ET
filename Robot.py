import os
import argparse
from enum import Enum

import numpy as np

from Roadmap import Roadmap
import random
import csv
from dataclasses import dataclass, field
import arcade.color

@dataclass
class Geometry:
    shape: int = 0  # 0=CIRCLE, 1=RECTANGULAR, 2=POLYGON
    length: int = -1
    width: int = -1


@dataclass
class Configuration:
    x: int = -1
    y: int = -1
    th: int = -1

    v: int = -1
    w: int = -1
    a: int = -1
    d: int = -1


class Robot:
    def __init__(self, idx, species):

        self.idx = idx
        self.species = species  # carry or support
        self.roadmap = None
        self.color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))

        self.geo = Geometry
        self.config = Configuration
        self.has_pod = False

        # capabilities and constraints
        self.capabilities = np.zeros((1, 2))
        self.capacity = -1
        self.power = -1
        
        self.status = 'free'  # free, busy, travel, idle, broken, p&p

        self.zone_idx = -1
        self.tasks = set()
        self.task_idx = -1
        self.mission_idx = -1

        self.path = None
        self.segments = None
        self.commands = []

        # robot location on graphs. parameterized by edges
        self.edge_idx = -1
        self.sigma = 0  # [0, max_len]
        self.direction = -1  # right. discrete direction, up,down,right,left

        self.reassign = False
        self.reschedule = False
        self.replan = False
        self.replan_local = False

    def set_task(self, task_idx):
        self.task_idx = task_idx
        self.status = "busy"
        self.replan = True

    def setPath(self, path):
        self.path = path
        self.sigma = 0.0
        self.edge_index = 0
        self.node_index = 0
        self.max_len = sum(path['lengths'])
        self.max_node_index = len(path['vertices'])
        self.accumulated_length = 0
        self.replan = False

    def stateTransition(self):
        pass