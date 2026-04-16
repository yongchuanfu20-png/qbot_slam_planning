import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FuncAnimation
import heapq
from typing import Tuple, List, Set, Dict
import copy
import os
import math

# ----------prob: where to stop when not find the goal, obstacle indication


class DsLite:
    def __init__(self,grid,start,goal):
        grid = np.array(grid)
        self.grid_w = grid.shape[0]
        self.grid_h = grid.shape[1]

        self.grid = grid
        self.start = start
        self.goal = goal

        self.priority_u = []
        self.rhs = {}
        self.g = {}

        self.path = []

        for x in range(self.grid_w):
            for y in range(self.grid_h):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.g[goal] = 0
        self.rhs[goal] = 0

        heapq.heappush(self.priority_u, ((0, 0), self.goal))

    def get_path(self):
        return self.path

    # -----------------inner exe function,generate and update cost map
    def exe_func(self):

        while self.priority_u and (self.priority_u[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):

            k_old, node = heapq.heappop(self.priority_u)
            neighbors = self._get_neighbors(node)
            k_new = self.calculate_key(node)

            if k_old < k_new:
                heapq.heappush(self.priority_u, (k_new, node))
            elif self.g[node]> self.rhs[node]:
                self.g[node] = self.rhs[node]
                for neighbor in neighbors:
                    self.update_vertex(neighbor)
            else:
                self.g[node] = float('inf')
                for neighbor in neighbors + [node]:
                    self.update_vertex(neighbor)


    def find_extract_path(self):
        # ----------initializing-------------
        self.exe_func()
        min_cost = float('inf')

        path = []
        path.append(self.start)
        current_node = self.start


        # ----------------path extract----------
        while current_node != self.goal:
            neighbors = self._get_neighbors(current_node)
            for neighbor in neighbors:
                total_cost = self.cost(neighbor,current_node) + self.g[neighbor]
                if neighbor == self.goal:
                    path.append(neighbor)
                    self.path = path
                    return path
                elif total_cost <= min_cost and total_cost != float('inf'):
                    min_cost = total_cost
                    current_node = neighbor
                    path.append(current_node)




    def _get_neighbors(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = s[0] + dx, s[1] + dy
                if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                    neighbors.append((nx, ny))
        return neighbors

    def calculate_key(self,s):
            m = min(self.g[s], self.rhs[s])
            return (m + self._heuristic(self.start, s), m)



    def update_vertex(self, u: Tuple[int, int]):
        """Update vertex u"""
        if u != self.goal:
            min_cost = float('inf')
            for s in self._get_neighbors(u):
                total_cost = self.cost(u, s) + self.g[s]
                if total_cost < min_cost:
                    min_cost = total_cost
            self.rhs[u] = min_cost

        # Remove u from queue if it exists
        self.priority_u = [(k, s) for k, s in self.priority_u if s != u]
        heapq.heapify(self.priority_u)

        # Add u to queue if inconsistent
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.priority_u, (self.calculate_key(u), u))


    def cost(self, s1: Tuple[int, int], s2: Tuple[int, int]) -> float:
        """Cost of moving from s1 to s2"""
        if self.grid[s1]==255:
            return float('inf')
        # Diagonal vs straight movement
        if abs(s1[0] - s2[0]) + abs(s1[1] - s2[1]) == 2:
            return np.sqrt(2)
        return 1.0

    def _heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


    def replan(self,grid_new):

        obstacles_new = grid_new - self.grid
        self.grid = np.array(grid_new)

        obstacles_list = []

        for r in range(len(obstacles_new)):
            for c in range(len(obstacles_new[0])):
                if obstacles_new[r][c] == 255:
                    obstacles_list.append((r, c))

        # for obstacle in obstacles_list:
        #     self.g[obstacle] = float('inf')
        #     self.rhs[obstacle] = float('inf')

#         update vertex and cost map
        for obs in obstacles_list:
            for s in self._get_neighbors(obs) + [obs]:
                self.update_vertex(s)

        path = self.find_extract_path()
        return path


