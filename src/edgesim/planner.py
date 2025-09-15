from __future__ import annotations
from typing import List, Tuple, Set
import heapq
import math

Grid = List[List[int]]  # 0=free, 1=blocked

def _neighbors(x: int, y: int, w: int, h: int):
	for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
		nx, ny = x + dx, y + dy
		if 0 <= nx < w and 0 <= ny < h:
			yield nx, ny

def astar(grid: Grid, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
	w, h = len(grid[0]), len(grid)
	def h_cost(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
	openq = [(0.0, start)]
	g = {start: 0.0}
	parent = {start: None}
	closed: Set[Tuple[int,int]] = set()

	while openq:
		_, cur = heapq.heappop(openq)
		if cur in closed: continue
		if cur == goal:
			path = []
			while cur is not None:
				path.append(cur)
				cur = parent[cur]
			return list(reversed(path))
		closed.add(cur)
		cx, cy = cur
		for nx, ny in _neighbors(cx, cy, w, h):
			if grid[ny][nx] == 1:  # blocked
				continue
			ng = g[(cx, cy)] + h_cost((cx, cy), (nx, ny))
			if (nx, ny) not in g or ng < g[(nx, ny)]:
				g[(nx, ny)] = ng
				parent[(nx, ny)] = (cx, cy)
				f = ng + h_cost((nx, ny), goal)
				heapq.heappush(openq, (f, (nx, ny)))
	return []

def rasterize_occupancy(map_size: Tuple[float,float],
                        obstacles: List[Tuple[float,float,float,float]],
                        res: float = 0.2) -> tuple[Grid, float]:
	"""Create a coarse occupancy grid from AABB obstacles (x0,y0,x1,y1)."""
	W = int(map_size[0] / res) + 1
	H = int(map_size[1] / res) + 1
	grid: Grid = [[0 for _ in range(W)] for _ in range(H)]
	for (x0, y0, x1, y1) in obstacles:
		ix0, iy0 = int(x0 / res), int(y0 / res)
		ix1, iy1 = int(x1 / res), int(y1 / res)
		for j in range(max(0, iy0), min(H, iy1 + 1)):
			for i in range(max(0, ix0), min(W, ix1 + 1)):
				grid[j][i] = 1
	return grid, res

# ---- NEW: grid inflation by radius (in cells) ----
def inflate_grid(grid: Grid, inflate_cells: int) -> Grid:
	"""Binary-dilate obstacles by a square structuring element of radius `inflate_cells`."""
	if inflate_cells <= 0:
		# Return a shallow copy to avoid accidental in-place edits
		return [row[:] for row in grid]
	H, W = len(grid), len(grid[0]) if grid else 0
	out: Grid = [[0 for _ in range(W)] for _ in range(H)]
	r = inflate_cells
	for y in range(H):
		y0 = max(0, y - r); y1 = min(H - 1, y + r)
		for x in range(W):
			x0 = max(0, x - r); x1 = min(W - 1, x + r)
			blocked = False
			for jj in range(y0, y1 + 1):
				if blocked: break
				for ii in range(x0, x1 + 1):
					if grid[jj][ii] == 1:
						blocked = True
						break
			out[y][x] = 1 if blocked else 0
	return out
