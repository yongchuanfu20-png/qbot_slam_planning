import heapq
import math
from collections import defaultdict

INF = float("inf")


class DStarLite:
    def __init__(self, start, goal, grid_map, obstacle_value=255):
        self.start = start
        self.goal = goal
        self.map = [row[:] for row in grid_map]
        self.obstacle_value = obstacle_value

        self.height = len(grid_map)
        self.width = len(grid_map[0]) if self.height > 0 else 0

        self.g = defaultdict(lambda: INF)
        self.rhs = defaultdict(lambda: INF)
        self.km = 0.0

        self.open_heap = []
        self.open_best = {}

        self.rhs[self.goal] = 0.0
        self._push(self.goal, self._calculate_key(self.goal))

    # ---------- Public API ----------

    def get_path(self):
        if not self._valid(self.start) or not self._valid(self.goal):
            return []
        if self._is_blocked(self.start) or self._is_blocked(self.goal):
            return []

        self._compute_shortest_path()
        return self._extract_path()

    def replan(self, start, new_map):
        if len(new_map) != self.height or len(new_map[0]) != self.width:
            raise ValueError("new_map must have the same size as the original map")

        old_map = self.map
        old_start = self.start

        self.start = start
        self.map = [row[:] for row in new_map]
        self.km += self._heuristic(old_start, self.start)

        changed = []
        for y in range(self.height):
            for x in range(self.width):
                if old_map[y][x] != self.map[y][x]:
                    changed.append((x, y))

        for u in changed:
            self._update_vertex(u)
            for s in self._pred(u):
                self._update_vertex(s)

        if not self._valid(self.start) or self._is_blocked(self.start):
            return []
        if self._is_blocked(self.goal):
            return []

        self._compute_shortest_path()
        return self._extract_path()

    # ---------- Internal helpers ----------

    def _valid(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def _is_blocked(self, node):
        x, y = node
        return self.map[y][x] >= self.obstacle_value

    def _heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    def _all_dirs(self, node):
        x, y = node
        candidates = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
            (x + 1, y + 1), (x + 1, y - 1), (x - 1, y + 1), (x - 1, y - 1),
        ]
        return [n for n in candidates if self._valid(n)]

    def _succ(self, node):
        if self._is_blocked(node):
            return []

        x, y = node
        out = []

        for nx, ny in self._all_dirs(node):
            nxt = (nx, ny)
            if self._is_blocked(nxt):
                continue

            dx = nx - x
            dy = ny - y

            # prevent corner cutting on diagonal moves
            if dx != 0 and dy != 0:
                side1 = (x + dx, y)
                side2 = (x, y + dy)
                if self._is_blocked(side1) or self._is_blocked(side2):
                    continue

            out.append(nxt)

        return out

    def _pred(self, node):
        # same as successors for an undirected grid
        return self._succ(node)

    def _cost(self, a, b):
        if not self._valid(a) or not self._valid(b):
            return INF
        if self._is_blocked(a) or self._is_blocked(b):
            return INF

        ax, ay = a
        bx, by = b
        dx = abs(ax - bx)
        dy = abs(ay - by)

        if dx == 1 and dy == 1:
            # still prevent corner cutting
            side1 = (ax + (bx - ax), ay)
            side2 = (ax, ay + (by - ay))
            if self._is_blocked(side1) or self._is_blocked(side2):
                return INF
            return math.sqrt(2)

        if dx + dy == 1:
            return 1.0

        return INF

    def _calculate_key(self, s):
        m = min(self.g[s], self.rhs[s])
        return (m + self._heuristic(self.start, s) + self.km, m)

    def _push(self, node, key):
        self.open_best[node] = key
        heapq.heappush(self.open_heap, (key, node))

    def _remove_if_stale(self):
        while self.open_heap:
            key, node = self.open_heap[0]
            if self.open_best.get(node) == key:
                return
            heapq.heappop(self.open_heap)

    def _top_key(self):
        self._remove_if_stale()
        if not self.open_heap:
            return (INF, INF)
        return self.open_heap[0][0]

    def _pop(self):
        self._remove_if_stale()
        if not self.open_heap:
            return None

        key, node = heapq.heappop(self.open_heap)
        if self.open_best.get(node) == key:
            del self.open_best[node]
            return node
        return None

    def _update_vertex(self, u):
        if u != self.goal:
            best = INF
            for s in self._succ(u):
                best = min(best, self._cost(u, s) + self.g[s])
            self.rhs[u] = best

        if self.g[u] != self.rhs[u]:
            self._push(u, self._calculate_key(u))
        else:
            if u in self.open_best:
                del self.open_best[u]

    def _compute_shortest_path(self):
        while (
            self._top_key() < self._calculate_key(self.start)
            or self.rhs[self.start] != self.g[self.start]
        ):
            k_old = self._top_key()
            u = self._pop()
            if u is None:
                break

            k_new = self._calculate_key(u)

            if k_old < k_new:
                self._push(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self._pred(u):
                    self._update_vertex(p)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for p in self._pred(u):
                    self._update_vertex(p)

    def _extract_path(self):
        if self.g[self.start] == INF:
            return []

        path = [self.start]
        current = self.start
        visited = {current}

        while current != self.goal:
            best_next = None
            best_cost = INF

            for s in self._succ(current):
                c = self._cost(current, s) + self.g[s]
                if c < best_cost:
                    best_cost = c
                    best_next = s

            if best_next is None or best_cost == INF:
                return []

            if best_next in visited:
                return []

            path.append(best_next)
            visited.add(best_next)
            current = best_next

            if len(path) > self.width * self.height:
                return []

        return path