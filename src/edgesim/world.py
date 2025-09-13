from __future__ import annotations
from typing import Dict, Any, Tuple, List
import pybullet as p
import pybullet_data

def _mk_box(half_extents, rgba=(0.8,0.8,0.8,1.0)):
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
	return vis, col

def build_world(scn: Dict[str, Any], use_gui: bool = False) -> Dict[str, Any]:
	"""Create a simple 2.5D warehouse scene.
	- Base floor (dry friction)
	- Walls as thin boxes
	- Aisles (visual only)
	- Optional 'wet patch' slab with low friction in each traction zone
	Returns IDs and helpful info.
	"""
	client = p.connect(p.GUI if use_gui else p.DIRECT)
	p.resetSimulation()
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setGravity(0, 0, -9.81)

	# Ground plane (visual + collision)
	plane_id = p.loadURDF("plane.urdf")
	# Dry friction on plane
	p.changeDynamics(plane_id, -1, lateralFriction=0.9, rollingFriction=0.0, spinningFriction=0.0)

	# World bounds & walls (very thin tall boxes)
	layout = scn["layout"]
	walls = []
	for wall in layout.get("walls", []):
		# (Placeholder if you later want arbitrary walls from 'wall')
		pass

	# A couple of perimeter walls to keep the robot in-bounds
	Lx, Ly = layout["map_size_m"]
	H = 0.5
	# left
	vis,col = _mk_box([0.05, Ly/2, H], rgba=(0.2,0.2,0.2,1))
	w0 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[0.0, Ly/2, H])
	# right
	vis,col = _mk_box([0.05, Ly/2, H], rgba=(0.2,0.2,0.2,1))
	w1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx, Ly/2, H])
	# bottom
	vis,col = _mk_box([Lx/2, 0.05, H], rgba=(0.2,0.2,0.2,1))
	w2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx/2, 0.0, H])
	# top
	vis,col = _mk_box([Lx/2, 0.05, H], rgba=(0.2,0.2,0.2,1))
	w3 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx/2, Ly, H])
	walls = [w0,w1,w2,w3]

	# Wet patches: thin slabs with low friction over the plane
	patch_bodies: List[int] = []
	for patch in scn["hazards"].get("traction", []):
		x0,y0,x1,y1 = patch["zone"]
		cx, cy = (x0+x1)/2.0, (y0+y1)/2.0
		hx, hy = max(0.01,(x1-x0)/2.0), max(0.01,(y1-y0)/2.0)
		vis,col = _mk_box([hx, hy, 0.001], rgba=(0.2,0.6,1.0,0.35))  # light blue transparent
		bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
		                        basePosition=[cx, cy, 0.001])
		p.changeDynamics(bid, -1, lateralFriction=float(patch.get("mu", 0.45)))
		patch_bodies.append(bid)

	return {
		"client": client,
		"plane_id": plane_id,
		"walls": walls,
		"patches": patch_bodies,
		"bounds": (Lx, Ly),
	}

def spawn_human(radius: float = 0.25, length: float = 1.6, rgba=(0.2, 0.8, 0.2, 1.0)):
	"""Create a kinematic human as a vertical capsule. Returns body id."""
	col = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=length)
	vis = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=rgba)
	# Center of a Z-capsule is at z = radius + length/2 when touching the ground plane at z=0
	z_center = float(radius + 0.5 * length)  # 0.25 + 0.8 = 1.05 for defaults
	bid = p.createMultiBody(
		baseMass=0.0,  # kinematic
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[-10.0, -10.0, z_center],  # start off-stage but at correct height
	)
	return bid
