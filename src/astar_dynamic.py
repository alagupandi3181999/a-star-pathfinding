#!/usr/bin/env python3
'''astar_dynamic.py - improved for project submission

Features added:
- explicit start/goal validation
- admissible and non-admissible heuristics (octile, weighted_manhattan)
- detailed docstrings and comments
- simple CLI-friendly benchmark helper
'''

import math
import time
import random
from typing import List, Tuple, Dict, Optional, Callable

INF = float('inf')


class IndexedMinHeap:
    """Indexed min binary heap supporting decrease-key via a position map.
    Entries are tuples (key, tie, node_id). Lower (key, tie) sorts first.
    Complexity: push/pop O(log n), decrease-key O(log n), contains O(1).
    """

    def __init__(self):
        self.heap = []  # List[Tuple[float, float, int]]
        self.pos = {}   # Dict[int, int]

    def __len__(self):
        return len(self.heap)

    def _swap(self, i: int, j: int):
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]
        self.pos[self.heap[i][2]] = i
        self.pos[self.heap[j][2]] = j

    def _sift_up(self, idx: int):
        while idx > 0:
            parent = (idx - 1) >> 1
            if self.heap[idx][:2] < self.heap[parent][:2]:
                self._swap(idx, parent)
                idx = parent
            else:
                break

    def _sift_down(self, idx: int):
        n = len(self.heap)
        while True:
            left = (idx << 1) + 1
            right = left + 1
            smallest = idx
            if left < n and self.heap[left][:2] < self.heap[smallest][:2]:
                smallest = left
            if right < n and self.heap[right][:2] < self.heap[smallest][:2]:
                smallest = right
            if smallest != idx:
                self._swap(idx, smallest)
                idx = smallest
            else:
                break

    def push(self, key: float, tie: float, node_id: int):
        entry = (key, tie, node_id)
        if node_id in self.pos:
            idx = self.pos[node_id]
            old = self.heap[idx]
            if entry[:2] < old[:2]:
                self.heap[idx] = entry
                self._sift_up(idx)
        else:
            idx = len(self.heap)
            self.heap.append(entry)
            self.pos[node_id] = idx
            self._sift_up(idx)

    def pop(self):
        if not self.heap:
            raise IndexError("pop from empty heap")
        top = self.heap[0]
        last = self.heap.pop()
        del self.pos[top[2]]
        if self.heap:
            self.heap[0] = last
            self.pos[last[2]] = 0
            self._sift_down(0)
        return top

    def contains(self, node_id: int) -> bool:
        return node_id in self.pos

    def decrease_key(self, node_id: int, new_key: float, new_tie: float):
        # wrapper; push already handles updating existing entries
        self.push(new_key, new_tie, node_id)

    def remove(self, node_id: int):
        if node_id not in self.pos:
            return
        idx = self.pos[node_id]
        last = self.heap.pop()
        del self.pos[node_id]
        if idx < len(self.heap):
            self.heap[idx] = last
            self.pos[last[2]] = idx
            self._sift_up(idx)
            self._sift_down(idx)


class GridGraph:
    """2D grid graph. grid[r][c] == 0 => free, 1 => blocked.
    Supports directed dynamic edge weights via set_edge_weight(u,v,w).
    """
    def __init__(self, rows: int, cols: int, allow_diagonal: bool = True):
        self.rows = rows
        self.cols = cols
        self.N = rows * cols
        self.allow_diagonal = allow_diagonal
        self.grid = [[0 for _ in range(cols)] for _ in range(rows)]
        self.edge_weights = {}  # Dict[(u,v), w]

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def coord_to_id(self, r: int, c: int) -> int:
        return r * self.cols + c

    def id_to_coord(self, idx: int) -> Tuple[int, int]:
        return divmod(idx, self.cols)

    def set_block(self, r: int, c: int, blocked: bool = True):
        if not self.in_bounds(r, c):
            raise IndexError("set_block out of bounds")
        self.grid[r][c] = 1 if blocked else 0

    def set_edge_weight(self, u: int, v: int, w: float):
        self.edge_weights[(u, v)] = w

    def get_edge_weight(self, u: int, v: int, default_cost: float = 1.0) -> float:
        return self.edge_weights.get((u, v), default_cost)

    def neighbors(self, node_id: int):
        r, c = self.id_to_coord(node_id)
        result = []
        steps = [(-1,0),(1,0),(0,-1),(0,1)]
        diag_steps = [(-1,-1),(-1,1),(1,-1),(1,1)]
        for dr,dc in steps:
            nr,nc = r+dr, c+dc
            if self.in_bounds(nr,nc) and self.grid[nr][nc] == 0:
                nid = self.coord_to_id(nr,nc)
                w = self.get_edge_weight(node_id, nid, 1.0)
                result.append((nid, w))
        if self.allow_diagonal:
            for dr,dc in diag_steps:
                nr,nc = r+dr, c+dc
                if not (self.in_bounds(nr,nc) and self.grid[nr][nc] == 0):
                    continue
                # corner cutting prevention
                if self.grid[r+dr][c] == 1 or self.grid[r][c+dc] == 1:
                    continue
                nid = self.coord_to_id(nr,nc)
                w = self.get_edge_weight(node_id, nid, math.sqrt(2))
                result.append((nid, w))
        return result


def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def octile(a,b, straight_cost=1.0, diag_cost=math.sqrt(2)):
    dx = abs(a[0]-b[0]); dy = abs(a[1]-b[1])
    return (min(dx,dy)*diag_cost) + (abs(dx-dy)*straight_cost)

def weighted_manhattan(a,b, w=1.5):
    # Non-admissible if w > 1.0 on standard uniform-cost grid.
    return w * (abs(a[0]-b[0]) + abs(a[1]-b[1]))


Heuristic = Callable[[Tuple[int,int], Tuple[int,int]], float]

class AStarEngine:
    def __init__(self, graph: GridGraph):
        self.graph = graph
        self.g = [INF] * graph.N
        self.parent = [-1] * graph.N
        self.closed = [False] * graph.N
        self.open_heap = IndexedMinHeap()

    def reset(self):
        self.g = [INF] * self.graph.N
        self.parent = [-1] * self.graph.N
        self.closed = [False] * self.graph.N
        self.open_heap = IndexedMinHeap()

    def reconstruct_path(self, goal_id: int):
        path = []
        cur = goal_id
        while cur != -1:
            path.append(cur)
            cur = self.parent[cur]
        path.reverse()
        return path

    def search(self, start_id: int, goal_id: int,
               heuristic_func: Heuristic,
               heuristic_weight: float = 1.0,
               tie_break: bool = True,
               max_expansions: int = 10_000_000):
        # validate
        if not (0 <= start_id < self.graph.N) or not (0 <= goal_id < self.graph.N):
            return {"path": None, "cost": INF, "expanded": 0, "time_s": 0.0, "error": "start/goal out of bounds"}
        sc = self.graph.id_to_coord(start_id); gc = self.graph.id_to_coord(goal_id)
        if self.graph.grid[sc[0]][sc[1]] == 1 or self.graph.grid[gc[0]][gc[1]] == 1:
            return {"path": None, "cost": INF, "expanded": 0, "time_s": 0.0, "error": "start/goal blocked"}
        if start_id == goal_id:
            return {"path":[start_id], "cost":0.0, "expanded":0, "time_s":0.0}

        self.reset()
        self.g[start_id] = 0.0
        h0 = heuristic_func(sc, gc)
        f0 = self.g[start_id] + heuristic_weight * h0
        tie0 = -self.g[start_id] if tie_break else 0.0
        self.open_heap.push(f0, tie0, start_id)

        expanded = 0
        start_time = time.perf_counter()

        while len(self.open_heap) > 0:
            f, tie, current = self.open_heap.pop()
            if current == goal_id:
                t = time.perf_counter() - start_time
                return {"path": self.reconstruct_path(goal_id), "cost": self.g[goal_id], "expanded": expanded, "time_s": t}
            if self.closed[current]:
                continue
            self.closed[current] = True
            expanded += 1
            for nbr, move_cost in self.graph.neighbors(current):
                tentative = self.g[current] + move_cost
                if tentative < self.g[nbr]:
                    self.g[nbr] = tentative
                    self.parent[nbr] = current
                    if self.closed[nbr]:
                        # reopen if improved
                        self.closed[nbr] = False
                    h = heuristic_func(self.graph.id_to_coord(nbr), gc)
                    f_n = tentative + heuristic_weight * h
                    tie_n = -tentative if tie_break else 0.0
                    self.open_heap.push(f_n, tie_n, nbr)
        t = time.perf_counter() - start_time
        return {"path": None, "cost": INF, "expanded": expanded, "time_s": t}


# Helper to create random grid with obstacle probability
def create_random_grid(rows, cols, obstacle_prob, allow_diagonal=True, rng=None):
    if rng is None:
        rng = random.Random()
    g = GridGraph(rows, cols, allow_diagonal)
    for r in range(rows):
        for c in range(cols):
            if rng.random() < obstacle_prob:
                g.set_block(r,c,True)
    return g

# small helper main for quick manual runs
if __name__ == "__main__":
    print("This module is intended to be imported for tests/benchmarks.")
