import argparse
import os
import re


class Solution:
    def __init__(self, solver, filename):
        self.robot_num = 0
        self.max_timestep = 0
        self.paths = {}
        self.configs = {}
        self.goals = {}
        self.pixel_paths = {}
        if solver == "lacam":
            self.read_lacam_results(filename)
        else:
            self.read_mapf_results(filename)

    def read_lacam_results(self, filename):
        if not os.path.isfile(filename):
            print("Path file not found!")
            exit(-1)
        configs_ls = open(filename, 'r').readlines()
        configs_raw = [elem.replace('\n', '') for elem in configs_ls]
        self.max_timestep = len(configs_raw)

        for elem in configs_raw:
            k = int(elem.split(':')[0])
            config = elem.split(':')[1]
            new_str = config.strip()
            new_str = config.strip("(,)")
            new_str = new_str.strip("(,)")            
            locations = new_str.split('),(')
            self.configs[k] = []
            for pair in locations:
                x, y = int(pair.split(',')[0]), int(pair.split(',')[1])
                self.configs[k].append((x, y))
        self.robot_num = len(self.configs[0])
        for i in range(self.robot_num):
            self.paths[i] = []
            self.pixel_paths[i] = []
            self.goals[i] = self.configs[self.max_timestep-1][i]
            for elem in self.configs.values():
                self.paths[i].append(elem[i])                


    def read_mapf_results(self, filename):
        if not os.path.isfile(filename):
            print("Path file not found!")
            exit(-1)
        configs_ls = open(filename, 'r').readlines()
        paths_raw = [elem.replace('\n', '') for elem in configs_ls]
        self.robot_num = len(paths_raw)

        self.max_timestep = 0
        for elem in paths_raw:
            idx = int(elem.split(' ')[1].split(':')[0])  # 获取机器人编号
            path = elem.split(':')[1]  # 获取路径
            path = path.strip()
            path = path.strip('->')  # 移除尾部的箭头
            locations = path.split(')->(')  # 分割每个坐标
            locations = [loc.strip('()') for loc in locations]  # 去掉括号

            self.paths[idx] = []
            self.pixel_paths[idx] = []
            for pair in locations:
                r, c = int(pair.split(',')[0]), int(pair.split(',')[1])
                self.paths[idx].append((c, r))
            self.max_timestep = max(self.max_timestep, len(locations))
            self.goals[idx] = self.paths[idx][-1]

        for i in range(self.robot_num):
            for j in range(len(self.paths[i]), self.max_timestep):
                self.paths[i].append(self.paths[i][-1])

        for k in range(self.max_timestep):
            self.configs[k] = []
            for i in range(self.robot_num):
                self.configs[k].append(self.paths[i][k])

    '''        
    def read_mapf_results(self, filename):
        if not os.path.isfile(filename):
            print("Path file not found!")
            exit(-1)
        configs_ls = open(filename, 'r').readlines()
        paths_raw = [elem.replace('\n', '') for elem in configs_ls]
        self.robot_num = len(paths_raw)

        self.max_timestep = 0
        for elem in paths_raw:
            idx = int(elem.split(':')[0])
            path = elem.split(':')[1]
            path = path.strip()
            path = path.strip("(,)")
            path = path.strip("(,)")            
            locations = path.split('),(')
            self.paths[idx] = []
            self.pixel_paths[idx] = []            
            for pair in locations:
                r, c = int(pair.split(',')[0]), int(pair.split(',')[1])
                self.paths[idx].append((c, r))
            self.max_timestep = max(self.max_timestep, len(locations))
            self.goals[idx] = self.paths[idx][-1]
        
        for i in range(self.robot_num):
            for j in range(len(self.paths[i]),self.max_timestep):
                self.paths[i].append(self.paths[i][-1])

        for k in range(self.max_timestep):
            self.configs[k] = []
            for i in range(self.robot_num):
                self.configs[k].append(self.paths[i][k])
    '''