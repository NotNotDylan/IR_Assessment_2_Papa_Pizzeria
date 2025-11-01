from manipulatable_object import ManipulatableObject
from pathlib import Path
import os
import swift
import roboticstoolbox as rtb
from ir_support.robots.UR3 import UR3
import ir_support.robots.UR3 as ur3_mod
from Robots.ABB_IRB_2400.IRB_2400 import IRB2400
from Robots.IRB_4600.ABB_IRB_4600 import IRB_4600
from spatialmath import SE3
from spatialmath.base import *
from math import pi
import time
from spatialgeometry import Sphere, Arrow, Mesh, Cuboid, Cylinder
import spatialgeometry as geometry
import spatialmath.base as spb
import numpy as np
import threading
from ir_support import RectangularPrism, line_plane_intersection, CylindricalDHRobotPlot

# =====================================================================================
# PATH SETUP
# =====================================================================================
script_dir = Path(__file__).parent.resolve()

# this is only to keep Swift happy about C: → /.../
def swift_prefix_from_windows_path(p: Path) -> str:
    s = p.as_posix()
    if ":" in s:
        s = s.split(":", 1)[1]
    return s.lstrip("/") + ("/" if not s.endswith("/") else "")

# move cwd to drive root (like you had)
swift_root = Path(script_dir.drive + "/").resolve()
os.chdir(swift_root)

# your existing prefixes
env_prefix = swift_prefix_from_windows_path(script_dir / "Environment")
pizza_prefix = swift_prefix_from_windows_path(script_dir / "Pizza's")

# repo root as Swift sees it, e.g. "AkaalBranch/IR_Assessment_2_Papa_Pizzeria/"
repo_prefix = swift_prefix_from_windows_path(script_dir)

# this is the IMPORTANT one – what we want Swift to actually retrieve from:
LOCAL_ROBOT_ROOT = repo_prefix + "Robots/"          # → ".../Robots/"

# then specific robot folders, all under LOCAL_ROBOT_ROOT
UR3_MESH_PREFIX      = LOCAL_ROBOT_ROOT + "UR3/"
IRB4600_MESH_PREFIX  = LOCAL_ROBOT_ROOT + "IRB_4600/"
IRB2400_MESH_PREFIX  = LOCAL_ROBOT_ROOT + "ABB_IRB_2400/"

# =====================================================================================
# HELPERS TO RETARGET MESHES
# =====================================================================================

def retarget_robot_meshes(robot, folder_prefix: str):
    """
    Change meshes on the *robot object itself* so they point to our repo, not site-packages.
    We only keep the filename (base_ur3.dae) and prepend our folder_prefix.
    """
    for link in getattr(robot, "links", []):
        geoms = getattr(link, "geometry", None)
        if not geoms:
            continue
        if not isinstance(geoms, (list, tuple)):
            geoms = [geoms]
        for g in geoms:
            if isinstance(g, Mesh) and getattr(g, "filename", None):
                tail = Path(str(g.filename)).name
                g.filename = folder_prefix + tail

    # sometimes robots have a top-level geometry as well – fix that too
    for attr in ("geometry", "_geometry", "visual"):
        g = getattr(robot, attr, None)
        if isinstance(g, Mesh) and getattr(g, "filename", None):
            tail = Path(str(g.filename)).name
            g.filename = folder_prefix + tail

def remap_swift_mesh_paths(env, local_robot_root: str):
    """
    ***THIS is the bit you were missing.***
    After robots are added to Swift, Swift keeps its own list (env.swift_objects).
    Some of those still point to:
        C:/Users/.../site-packages/ir_support/robots/UR3/base_ur3.dae
    We walk that list and rewrite ONLY those to our local repo path.
    """
    objs = getattr(env, "swift_objects", [])
    for i in range(0, len(objs), 2):      # [obj, pose, obj, pose, ...]
        obj = objs[i]
        if isinstance(obj, Mesh) and getattr(obj, "filename", None):
            raw = str(obj.filename).replace("\\", "/")

            # match both installed-path and package-ish path
            if "/site-packages/ir_support/robots/" in raw:
                rel = raw.split("/site-packages/ir_support/robots/", 1)[1]
                obj.filename = local_robot_root + rel
            elif "/ir_support/robots/" in raw:
                rel = raw.split("/ir_support/robots/", 1)[1]
                obj.filename = local_robot_root + rel
            # else: leave other meshes (your Environment, Pizza, etc.) alone


# =====================================================================================
# WORLD
# =====================================================================================
class World:
    """Simulation world: handles environment launch, and loading of robots, objects, and safety elements."""
    def __init__(self):
        self.env = swift.Swift()
        self.robot_test = None
        self.robot1 = None     # UR3
        self.robot2 = None
        self.robot3 = IRB_4600()
        self.robot4 = IRB2400()
        self.conveyors = []
        self.motorbike = None
        self.safety_barriers = []
        self.objects = []

        self.last_update = time.time()
        self.plates = []
        self.x = 4.05
        self.y = 3.6
        self.z = 0.99
        self.loop = 1
        self.loop2 = 1
        self.first_period = 0.25
        self.last_time = 0.25
        self.displacement = 0.1
        self.last_time_pizza = 0.25
        self.x_pizza = 3.5
        self.y_pizza = 3.6
        self.z_pizza = 1.015
        self.last_pizza_time = float(self.env.sim_time)
        self.pos1 = SE3(4.6, 3.6, 1.015)
        self.pos2 = SE3(6.5, 3.6, 1.015)
        self.pos3 = SE3(8.7, 3.6, 1.015)
        self.pos = 1
        self.sauce_placed = False
        self.collision_opacity = 0.5
        self.xp = 7.5
        self.yp = 6.5
        self.meshes = []

    def launch(self, environment_objects: bool = False):
        self.env.launch(realtime=True)

        if environment_objects:
            # --- all your environment meshes unchanged ---
            wall = Mesh(filename=env_prefix + "Wall.stl", color=(1,1,1,1))
            self.env.add(wall)

            floor = Mesh(filename=env_prefix + "Floor.stl", color=(1,1,0,1))
            self.env.add(floor)

            Conveyer_One = Mesh(filename=env_prefix + "First_Conveyer2.0.3.stl", color=(0.5,0.5,0.5,1))
            self.env.add(Conveyer_One)

            Table = Mesh(filename=env_prefix + "Table.stl", color=(0.588, 0.294, 0.0))
            self.env.add(Table)

            Pizza_Oven = Mesh(filename=env_prefix + "Pizza_Oven2.stl", color=(0.886,0.447,0.357,1))
            self.env.add(Pizza_Oven)

            Light_Fence_Post = Mesh(filename=env_prefix + "Light_Fence_Post.stl", color=(0.1,0.1,0.1,1))
            self.env.add(Light_Fence_Post)

            Pillar_1 = Mesh(filename=env_prefix + "Pillar.stl",
                            color=(0.5,0.5,0.5,1), pose=SE3(5.8486, 6.4944, 0), scale=[1,1,0.5])
            self.env.add(Pillar_1)

            Pillar_2 = Mesh(filename=env_prefix + "Pillar.stl",
                            color=(0.5,0.5,0.5,1), pose=SE3(9.72, 5.6, 0), scale=[1,1,0.5])
            self.env.add(Pillar_2)

            Pillar_3 = Mesh(filename=env_prefix + "Pillar.stl",
                            color=(0.5,0.5,0.5,1), pose=SE3(4.6, 4.05, 0))
            self.env.add(Pillar_3)

            Pillar_4 = Mesh(filename=env_prefix + "Pillar.stl",
                            color=(0.5,0.5,0.5,1), pose=SE3(6.52, 4.4, 0))
            self.env.add(Pillar_4)

            Toppings_Table = Mesh(filename=env_prefix + "Toppings_Table2.stl",
                                  color=(0.5,0.5,0.5,1), pose=SE3(0, 0, 0))
            self.env.add(Toppings_Table)

            self.plate = Mesh(filename=env_prefix + "Conveyor_Movement.stl",
                              pose=SE3(self.x, self.y, self.z),
                              color=(0.25,0.25,0.25,1), scale=[1,1,1])
            self.env.add(self.plate)

            self.plate2 = Mesh(filename=env_prefix + "Conveyor_Movement.stl",
                               pose=SE3(self.x+0.1, self.y, self.z),
                               color=(0.35,0.35,0.35,1), scale=[1,1,1])
            self.env.add(self.plate2)

            self.pizza = Mesh(filename=pizza_prefix + "Pizza_Base.stl",
                              pose=SE3(self.xp, self.yp, (0.475905*2)),
                              color=(0.90, 0.83, 0.70))
            self.env.add(self.pizza)

            self.sauce = Mesh(filename=pizza_prefix + "Pizza_Sauce.stl",
                              pose=SE3(self.xp, self.yp, (0.475905*2)+0.0075),
                              color=(0.698, 0.133, 0.133))
            self.env.add(self.sauce)

            self.cheese = Mesh(filename=pizza_prefix + "Pizza_Cheese.stl",
                               pose=SE3(self.xp, self.yp, 1.0125),
                               color=(1.0, 0.78, 0.24))

            self.olives = Mesh(filename=pizza_prefix + "Olives.stl",
                               pose=SE3(self.xp, self.yp, (0.475905*2)+0.0125),
                               color=(0.20, 0.20, 0.20))
            self.env.add(self.olives)

            self.ham = Mesh(filename=pizza_prefix + "Ham.stl",
                            pose=SE3(self.xp, self.yp, (0.475905*2)+0.0125),
                            color=(1.0, 0.71, 0.76))
            self.env.add(self.ham)

            self.pepperoni = Mesh(filename=pizza_prefix + "Pepperoni.stl",
                                  pose=SE3(self.xp, self.yp, (0.475905*2)+0.0125),
                                  color=(0.71, 0.20, 0.14))
            self.env.add(self.pepperoni)

            self.pineapple = Mesh(filename=pizza_prefix + "Pineapple.stl",
                                  pose=SE3(self.xp, self.yp, (0.475905*2)+0.0125),
                                  color=(1.0, 0.90, 0.39))
            self.env.add(self.pineapple)

            self.melted_cheese = Mesh(filename=pizza_prefix + "Melted_Cheese.stl",
                                      pose=SE3(self.xp, self.yp, (0.475905*2)+0.01241),
                                      color=(1.0, 0.78, 0.25))
            self.env.add(self.melted_cheese)

            self.box = Mesh(filename=pizza_prefix + "Pizza_Box.stl",
                            pose=SE3(self.xp, self.yp, 0.95181),
                            color=(1.0, 1.0, 1.0))
            self.env.add(self.box)

            self.motorbike = Mesh(filename=env_prefix + "Honda Hornet STL.stl",
                                  pose=SE3(3, 8.0, 0.0).A @ spb.trotz(pi),
                                  color=(0.8,0.8,0.8))
            self.env.add(self.motorbike)

            converyer_collision1 = Cuboid(scale=[1, 1.2, 1.5], base=SE3(3.5, 3.6, 0.75),
                                          color=(1,1,1,self.collision_opacity))
            self.env.add(converyer_collision1)

            converyer_collision2 = Cuboid(scale=[5, 0.4, 1], base=SE3(6.5, 3.6, 0.5),
                                          color=(1,1,1,self.collision_opacity))
            self.env.add(converyer_collision2)

            table_collision = Cuboid(scale=[1, 1.5, 1], base=SE3(7.5, 6.4935, 0.475905),
                                     color=(1,1,1,self.collision_opacity))
            self.env.add(table_collision)

            pizza_oven_collision = Cuboid(scale=[3,3,1], base=SE3(11.983, 7.8631, 0.5),
                                          color=(1,1,1,self.collision_opacity))
            self.env.add(pizza_oven_collision)

            light_fence_collision = Cuboid(scale=[4.22, 0.05, 3], base=SE3(5.21, 4.75, 1.5),
                                           color=(1,1,1,0.5))
            self.env.add(light_fence_collision)

            pillar1_collision = Cuboid(scale=[0.4,0.4,0.5], base=SE3(5.8486, 6.4944, 0.25),
                                       color=(1,1,1,self.collision_opacity))
            self.env.add(pillar1_collision)

            pillar2_collision = Cuboid(scale=[0.4,0.4,0.5], base=SE3(9.72, 5.6, 0.25),
                                       color=(1,1,1,self.collision_opacity))
            self.env.add(pillar2_collision)

            pillar3_collision = Cuboid(scale=[0.4,0.4,1], base=SE3(4.6, 4.05, 0.5),
                                       color=(1,1,1,self.collision_opacity))
            self.env.add(pillar3_collision)

            pillar4_collision = Cuboid(scale=[0.4,0.4,1], base=SE3(6.52, 4.4, 0.5),
                                       color=(1,1,1,self.collision_opacity))
            self.env.add(pillar4_collision)

            toppings_collision = Cuboid(scale=[1,0.25,1], base=SE3(6.5, 4, 0.5),
                                        color=(1,1,1,self.collision_opacity))
            self.env.add(toppings_collision)

            self.bounds_array = np.array([
                [3.0, 4.0, 3.0, 4.2, 0.0, 1.5],
                [4.0, 9.0, 3.4, 3.8, 0.0, 1.0],
                [7.0, 8.0, 5.7435, 7.2435, -0.024095, 0.975905],
                [10.483, 13.483, 6.3631, 9.3631, 0.0, 1.0],
                [3.10, 7.32, 4.725, 4.775, 0.0, 3.0],
                [5.6486, 6.0486, 6.2944, 6.6944, 0.0, 0.5],
                [9.52, 9.92, 5.4, 5.8, 0.0, 0.5],
                [4.4, 4.8, 3.85, 4.25, 0.0, 1.0],
                [6.32, 6.72, 4.2, 4.6, 0.0, 1.0],
                [6.0, 7.0, 3.875, 4.125, 0.0, 1.0]
            ])

    def setup_robots_and_objects(self):
        # ========== UR3 ==========
        self.robot1 = UR3()
        retarget_robot_meshes(self.robot1, UR3_MESH_PREFIX)
        self.robot1.base = SE3(4.6, 4.05, 1.0)
        self.robot1.add_to_env(self.env)

        # ========== IRB 4600 ==========
        self.robot3.base = SE3(9.72, 5.6, 0.5)
        retarget_robot_meshes(self.robot3, IRB4600_MESH_PREFIX)
        self.robot3.add_to_env(self.env)

        # ========== IRB 2400 ==========
        self.robot4.base = SE3(5.84, 6.49, 0.5)
        retarget_robot_meshes(self.robot4, IRB2400_MESH_PREFIX)
        self.robot4.add_to_env(self.env)

        # *** IMPORTANT ***: now that all robots are actually inside Swift, fix Swift's copies
        remap_swift_mesh_paths(self.env, LOCAL_ROBOT_ROOT)

        # (optional) collisions for 4600, keep as you had
        cyl_collision = CylindricalDHRobotPlot(self.robot3, cylinder_radius=0.05, color=(1,0,0,1))
        self.env.add(cyl_collision.create_cylinders())

    # ---------------------------------------------------------------------------------
    # the rest of your movement / pizza logic stays the same
    # ---------------------------------------------------------------------------------
    def add_object(self, obj):
        if self.env:
            self.env.add(obj.mesh)
        self.objects.append(obj)

    def remove_object(self, obj):
        if self.env:
            try:
                self.env.remove(obj.mesh)
            except Exception:
                pass
        if obj in self.objects:
            self.objects.remove(obj)

    def conveyorBelt_Movement(self, plate1, plate2, direction, period):
        self.t = float(self.env.sim_time)
        if self.t - self.last_time >= period:
            if self.loop == 1:
                plate1.T = plate1.T @ SE3((direction)*self.displacement, 0, 0).A
                plate2.T = plate2.T @ SE3((direction)*-(self.displacement), 0, 0).A
                self.loop = 2
            elif self.loop == 2:
                plate1.T = plate1.T @ SE3((direction)*-(self.displacement), 0, 0).A
                plate2.T = plate2.T @ SE3((direction)*self.displacement, 0, 0).A
                self.loop = 1
            self.last_time = self.t

    def pizza_movement(self, pos, period):
        self.t = float(self.env.sim_time)
        if not np.allclose((self.pizza.T), (SE3(pos).A)):
            if self.t - self.last_time_pizza >= period:
                self.pizza.T = self.pizza.T @ SE3(self.displacement, 0, 0).A
                self.sauce_movement()
                self.last_time_pizza = self.t
            self.conveyorBelt_Movement(self.plate, self.plate2, 1, 0.25)

    def pizza_timing(self, pause_1, pause_2):
        self.t = float(self.env.sim_time)
        match self.pos:
            case 1:
                self.last_pizza_time = float(self.env.sim_time)
                self.pizza_movement(pos=self.pos1, period=0.25)
                if np.allclose((self.pizza.T), (SE3(self.pos1).A)):
                    self.pos = 2
            case 2:
                if self.t - self.last_pizza_time >= pause_1:
                    self.pizza_movement(pos=self.pos2, period=0.25)
                if np.allclose((self.pizza.T), (SE3(self.pos2).A)):
                    self.last_pizza_time = float(self.env.sim_time)
                    self.pos = 3
            case 3:
                if self.t - self.last_pizza_time >= pause_2:
                    self.pizza_movement(pos=self.pos3, period=0.25)

    def sauce_placement(self):
        if np.allclose((self.pizza.T), (SE3(self.pos1).A)):
            self.sauce.T = self.pizza.T @ SE3(0,0,0.0075).A
            self.env.add(self.sauce)
            self.sauce_placed = True

    def sauce_movement(self):
        if self.sauce_placed:
            self.sauce.T = self.pizza.T @ SE3(0,0,0.0075).A
