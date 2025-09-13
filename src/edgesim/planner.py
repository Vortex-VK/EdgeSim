from __future__ import annotations
from typing import List, Tuple, Set
import heapq
import math

Grid = List[List[int]]  # 0=free,1=blocked

def _neighbors(x:int,y:int,w:int,h:int):
	for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
		nx,ny=x+dx,y+dy
		if 0<=nx<w and 0<=ny<h:
			yield nx,ny

def astar(grid: Grid, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
	w,h=len(grid[0]),len(grid)
	def h_cost(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])
	openq=[(0,start)]
	g={start:0.0}
	parent={start:None}
	closed:set[Tuple[int,int]]=set()

	while openq:
		_,cur=heapq.heappop(openq)
		if cur in closed: continue
		if cur==goal:
			path=[]
			while cur is not None:
				path.append(cur)
				cur=parent[cur]
			return list(reversed(path))
		closed.add(cur)
		for nx,ny in _neighbors(cur[0],cur[1],w,h):
			if grid[ny][nx]==1: continue
			ng=g[cur]+h_cost(cur,(nx,ny))
			if (nx,ny) not in g or ng<g[(nx,ny)]:
				g[(nx,ny)]=ng
				parent[(nx,ny)]=cur
				f=ng+h_cost((nx,ny),goal)
				heapq.heappush(openq,(f,(nx,ny)))
	return []

def rasterize_occupancy(map_size: Tuple[float,float], obstacles: List[Tuple[float,float,float,float]], res: float=0.2) -> tuple[Grid, float]:
	"""Create a coarse occupancy grid from AABB obstacles (x0,y0,x1,y1)."""
	W=int(map_size[0]/res)+1
	H=int(map_size[1]/res)+1
	grid=[[0 for _ in range(W)] for _ in range(H)]
	for (x0,y0,x1,y1) in obstacles:
		ix0,iy0=int(x0/res),int(y0/res)
		ix1,iy1=int(x1/res),int(y1/res)
		for j in range(max(0,iy0),min(H,iy1+1)):
			for i in range(max(0,ix0),min(W,ix1+1)):
				grid[j][i]=1
	return grid, res
