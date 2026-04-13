class DsLite:
    def __init__(self,grid,start,goal):
        self.grid_w = grid.shape[0]
        self.grid_h = grid.shape[1]

        self.grid = grid
        self.start = start
        self.goal = goal

        self.priority_u = []
        self.rhs = {}
        self.g = {}

        self.path = []

        for x in range(w):
            for y in range(h):
                self.g[(x, y)] = float('inf')
                self.rhs[(x, y)] = float('inf')

        self.g[goal] = 0
        self.rhs[goal] = 0

        heapq.heappush(self.priority_u, (0, 0), self.goal)

    def get_path(self):
        return self.path

    def exe_func(self):

        while self.U and (self.priority_u[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):

            k_old, node = heapq.heappop(self.priority_u)
            neighbors = _get_neighbors(node)
            k_new = calculatekey

            if key_old < k_new:
                heapq.heappush(self.priority_u, k_new, node)
            elif self.g[node]> self.rhs[node]:
                self.g[node] = self.rhs[node]
                for neighbor in neighbors:
                    self.update_vertex(neighbor)
            else:
                self.g[node] = float('inf')
                for neighbor in neighbors + [node]:
                    self.update_vertex(neighbor)

    def find_path(self):
        # ----------initializing-------------
        self.exe_func()
        min_cost = float('inf')
        path.append(self.start)
        current_node = self.start

        # ----------------path extract----------
        neighbors = _get_neighbors(current_node)

        for neighbor in neighbors:
            total_cost = cost(neighbor,current_node) + self.g[neighbor]
            if neighbor == self.goal:
                path.append(neighbor)
                self.path = path
                break
            if total_cost <= min_cost:
                min_cost = total_cost
                current_node = neighbor
                path.append(current_node)

        return path



    def replan_path(self):














    def _get_neighbors(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = s[0] + dx, s[1] + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    neighbors.append((nx, ny))
        return neighbors

    def calculate_key(self,):



    def _update_vertex(self, u: Tuple[int, int]):
        """Update vertex u"""
        if u != self.goal:
            min_cost = float('inf')
            for s in self._get_neighbors(u):
                cost = self._cost(u, s) + self.g[s]
                if cost < min_cost:
                    min_cost = cost
            self.rhs[u] = min_cost

        # Remove u from queue if it exists
        self.priority_u = [(k, s) for k, s in self.priority_u if s != u]
        heapq.heapify(self.priority_u)

        # Add u to queue if inconsistent
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.priority_u, (self._calculate_key(u), u))


    def cost(self, s1: Tuple[int, int], s2: Tuple[int, int]) -> float:
        """Cost of moving from s1 to s2"""
        if s2 in self.obstacles:
            return float('inf')
        # Diagonal vs straight movement
        if abs(s1[0] - s2[0]) + abs(s1[1] - s2[1]) == 2:
            return np.sqrt(2)
        return 1.0