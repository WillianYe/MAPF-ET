#!/usr/bin/env python3
"""
....................> x axis, column  (px)
.
.
.
.
v
y axis, row (py)
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
import numpy as np
import igraph as ig
from loguru import logger
import os
import argparse
import csv
import heapq


@dataclass
class Node:
    idx: int = -1
    linear_idx: int = -1
    x: int = -1  # col
    y: int = -1  # row
    tag: str = "."  # # {'@', '.', 'P', 'R', 'D', 'M', 'X', 'Y'}
    pair_idx: int = -1
    out_edges: list[tuple] = field(default_factory=list)
    in_edges: list[tuple] = field(default_factory=list)

    def __hash__(self) -> int:
        # 把list转成tuple，然后hash。内置的hash函数只支持不可变类型
        return hash(self.linear_idx)


@dataclass
class Obstacle:
    linear_idx: int = -1
    x: int = -1  # col
    y: int = -1  # row


@dataclass
class Edge:
    # directed edge. If ridx is not none, then it is undirected. The graph is a hybrid graph
    idx: int = -1
    ridx: int = -1  # reverse edge
    tail: int = -1
    head: int = -1
    weight: int = -1
    direction: int = -1  # 0,1,2,3 or [east,north,west,south]

    def __hash__(self) -> int:
        # 把list转成tuple，然后hash。内置的hash函数只支持不可变类型
        return hash(self.idx)


class Roadmap:
    def __init__(self, map_dir):
        self.map_dir = map_dir
        self.map_tag = None
        self.distribution = None
        self.height = 0
        self.width = 0

        self.obstacles = {}
        self.nodes = {}
        self.terminal_indices = []
        self.pickup_indices = []
        self.pickup_pair_indices = []
        self.delivery_indices = []
        self.delivery_pair_indices = []

        # for pareto maps with heat zones. 20% nodes are busy
        self.busy_pickup_indices = []
        self.busy_pickup_pair_indices = []

        self.edges = {}

        self.grid_map = np.ndarray
        self.grid_occupied = np.ndarray
        self.grid2node = np.ndarray
        self.heuristic_table = np.ndarray
        self.construct_map_from_mapfile()

    def construct_map_from_mapfile(self):
        if not os.path.isfile(self.map_dir):
            print("Map file not found!")
            exit(-1)
        words = self.map_dir.split('/')
        self.map_tag = words[-1].split('.')[0]
        map_ls = open(self.map_dir, 'r').readlines()
        grid_map_raw = [elem.replace('\n', '') for elem in map_ls]
        grid_map_raw.pop(0)
        grid_map_raw.pop(0)
        grid_map_raw.pop(0)
        grid_map_raw.pop(0)
        self.grid_map = grid_map_raw
        # print(grid_map_raw)
        self.height = len(grid_map_raw)
        self.width = len(grid_map_raw[0])
        print(f"{self.height},{self.width}")
        self.grid_map = np.full((self.height, self.width), '.')
        self.grid_occupied = np.full((self.height, self.width), False)
        self.grid2node = np.full((self.height, self.width), -1)
        for r in range(self.height):
            for c in range(self.width):

                # if grid_map_raw[r][c] in ['@', 'D']:
                self.grid_map[r, c] = grid_map_raw[r][c]
                if grid_map_raw[r][c] in ['@']:
                    self.grid_occupied[r, c] = True
                else:
                    self.grid_occupied[r, c] = False

        valid_chars = {'@', '.', 'T'}

        node_idx = 0
        for row, line in enumerate(grid_map_raw):
            for col, char in enumerate(line):
                y = row
                x = col
                linear_idx = self.linearize_coordinate(row, col)
                assert (char in valid_chars)
                if char in ['@', 'T']:
                    self.obstacles[linear_idx] = Obstacle(linear_idx, x, y)
                else:
                    self.nodes[node_idx] = Node(node_idx, linear_idx, x, y, char)
                    self.grid2node[row, col] = node_idx
                    node_idx += 1

        self.analyze_neighbors()
        self.analyze_edges()
        self.analyze_terminals()
        # self.compute_heuristic_table()

    def linearize_coordinate(self, row, col):
        return self.width * row + col

    def analyze_neighbors(self):
        edge_idx = 0
        for source, value in self.nodes.items():
            x = value.x
            y = value.y

            nbx = 0
            nby = 0
            for direction in range(4):
                if direction == 0:  # east
                    nbx = x + 1
                    nby = y
                elif direction == 1:  # north
                    nbx = x
                    nby = y - 1
                elif direction == 2:  # west
                    nbx = x - 1
                    nby = y
                elif direction == 3:  # south
                    nbx = x
                    nby = y + 1
                if 0 <= nbx < self.width and 0 <= nby < self.height:
                    sink = int(self.grid2node[nby, nbx])
                    if sink != -1:
                        self.nodes[source].out_edges.append((edge_idx, sink, direction))
                        self.nodes[sink].in_edges.append((edge_idx, source, direction))
                        edge_idx += 1

    def analyze_edges(self):
        for tail, v in self.nodes.items():
            x1 = v.x
            y1 = v.y
            if v.out_edges:
                for pair in v.out_edges:
                    eid = pair[0]
                    head = pair[1]
                    direction = pair[2]
                    x2 = self.nodes[head].x
                    y2 = self.nodes[head].y
                    reverse_id = -1
                    for temp in self.nodes[head].out_edges:
                        if temp[1] == tail:
                            reverse_id = temp[0]
                            break
                    self.edges[eid] = Edge(eid, reverse_id, tail, head, abs(x2 - x1) + abs(y2 - y1), direction)

    def analyze_terminals(self):
        for source, v in self.nodes.items():
            if v.tag in ['P', 'D', 'X']:
                candidate = []
                for vec in v.out_edges:
                    if vec[2] == 0 or vec[2] == 3:
                        candidate.append(vec[1])
                if candidate:
                    for sink in candidate:
                        if self.nodes[sink].tag in ['R', 'M', 'Y']:
                            self.nodes[source].pair_idx = sink
                            self.nodes[sink].pair_idx = source
                            break

    def compute_heuristic_table(self):
        self.heuristic_table = np.zeros((len(self.nodes), len(self.nodes)), dtype=np.int32)
        for start in self.nodes.keys():
            h_values = self.dijkstra(start)
            for goal in self.nodes.keys():
                if start == goal:
                    self.heuristic_table[start, goal] = 0
                else:
                    self.heuristic_table[start, goal] = h_values[goal]
                    # print("distance between node {} and node {} is:{}\n".format(start, goal, h_values[goal]))

    # def save_heuristic_table(self, file_name):
    #     with open(file_name, 'w', newline='') as file_obj:
    #         writer_obj = csv.writer(file_obj)
    #         for line in self.heuristic_table.values():
    #             writer_obj.writerow(line.values())

    # def load_heuristic_table(self, file_name):
    #     with open(file_name, newline='') as file_obj:
    #         reader_obj = csv.reader(file_obj)
    #         for i, row in enumerate(reader_obj):
    #             self.heuristic_table[i] = {}
    #             for j, x in enumerate(row):
    #                 self.heuristic_table[i][j] = int(x)

    def dijkstra(self, start):
        # Use Dijkstra to build a shortest-path tree rooted at the start location
        open_list = []
        closed_list = {}
        root_node = {'loc': start, 'cost': 0}
        heapq.heappush(open_list, (root_node['cost'], start, root_node))
        closed_list[start] = root_node
        while open_list:
            (cost, vid, curr) = heapq.heappop(open_list)
            for vec in self.nodes[vid].out_edges:
                cid = vec[1]
                child_cost = cost + self.edges[vec[0]].weight
                child = {'loc': cid, 'cost': child_cost}
                if cid in closed_list:
                    existing_node = closed_list[cid]
                    if existing_node['cost'] > child_cost:
                        closed_list[cid] = child
                        heapq.heappush(open_list, (child_cost, cid, child))
                else:
                    closed_list[cid] = child
                    heapq.heappush(open_list, (child_cost, cid, child))

        # build the heuristics table
        h_values = {}
        for vid, node in closed_list.items():
            h_values[vid] = node['cost']
        return h_values

    def print(self):
        for vid, v in self.nodes.items():
            print(v)
        for eid, e in self.edges.items():
            print(e)
        print(self.heuristic_table)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=str, help="select map dir")
    args = parser.parse_args()
    ln = Roadmap(args.map_dir)
    print(ln.map_tag)
    # ln.print()
