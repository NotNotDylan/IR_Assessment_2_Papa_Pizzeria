from manipulatable_object import ManipulatableObject

import swift
import roboticstoolbox as rtb
from ir_support import UR3
from ABB_IRB_2400.IRB_2400 import IRB2400
from IRB_4600.ABB_IRB_4600 import IRB_4600
from spatialmath import SE3
from spatialmath.base import *
from math import pi
import time 
from spatialgeometry import Sphere, Arrow, Mesh
import spatialgeometry as geometry
import os
import spatialmath.base as spb
from spatialmath import SE3

class World:
    """Simulation world: handles environment launch, and loading of robots, objects, and safety elements."""
    def __init__(self):
        self.env = swift.Swift()   # The Swift environment instance
        self.robot_test = None   # Robot I am using to temporaly test the GUI
        self.robot1 = IRB_4600()         # Robot performing sauce application
        self.robot2 = IRB2400()        # Robot performing topping placement
        self.robot3 = UR3()        # Robot handling oven loading/unloading
        self.robot4 = None         # Robot packaging and loading pizza on bike
        self.conveyors = []        # List of ConveyorBelt objects in the system
        self.motorbike = None      # The motorbike object (for delivery)
        self.safety_barriers = []  # Any safety barrier objects (e.g., fences, light curtain representation)
        self.objects = []          # List to hold other objects (pizzas, toppings, etc. currently in scene)
        # (Other environment attributes can be added as needed)
    
    def launch(self, environment_objects: bool = False):
        """Start the simulator and set up the static scene (floor, walls, etc.)."""
        # TODO: Create and launch the Swift environment
        # Example:
        # self.env = swift.Swift()
        # self.env.launch(realtime=True)
        # Add floor or background:
        # floor = geometry.Box(..., pose=SE3(0,0,-0.01), color=(0.5,0.5,0.5,1))
        # self.env.add(floor)
        
        self.env.launch(realtime=True)
        
        
        if environment_objects == True:
            # Walls
            wall = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Wall.stl"),
                        color=(1.0,1.0,1.0,1.0))
            self.env.add(wall)

            floor = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Floor.stl"),
                        color=(1.0,1.0,0.0,1.0))
            self.env.add(floor)

            Conveyer_One = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "First_Conveyer2.0.2.stl"),
                       color=(0.5,0.5,0.5,1.0))
            self.env.add(Conveyer_One)

            Table = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Table.stl"),
                        color=(0.588, 0.294, 0.0))
            self.env.add(Table)

            Pizza_Oven = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pizza_Oven2.stl"),
                        color=(0.886,0.447,0.357,1.0))
            self.env.add(Pizza_Oven)

            Light_Fence_Post = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Light_Fence_Post.stl"),
                        color=(0.1,0.1,0.1,1.0))
            self.env.add(Light_Fence_Post)
            #DYALN
            Pillar_1 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pillar.stl"),
                        color=(0.5,0.5,0.5,1.0), pose=SE3(5.8486, 6.4944, 0), scale=[1,1,0.5])
            self.env.add(Pillar_1)
            #AIDAN
            Pillar_2 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pillar.stl"),
                        color=(0.5,0.5,0.5,1.0), pose=SE3(9.72, 5.6, 0), scale=[1,1,0.5])
            self.env.add(Pillar_2)
            #UR3
            Pillar_3 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pillar.stl"),
                        color=(0.5,0.5,0.5,1.0), pose=SE3(4.6, 4.05, 0))
            self.env.add(Pillar_3)
            #AKAAL
            Pillar_4 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pillar.stl"),
                        color=(0.5,0.5,0.5,1.0), pose=SE3(6.52, 4.4, 0))
            self.env.add(Pillar_4)

            plate1 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyer_Top.stl"), 
                          pose = SE3(4.125 ,3.6 ,0.99).A @ trotz(pi/2))
            self.env.add(plate1)
            # Floor
            # Safety
            # Decorations
            # Oven
            pass
        
    
    def setup_robots_and_objects(self):
        """Load robots, conveyors, and objects into the environment."""
        # TODO: Initialize robot models (from RTB or custom DH parameters)
        # e.g., self.robot1 = rtb.models.DH.SomeRobotModel() or custom Robot class
        # Position robots in the scene (adjust base if needed, e.g., robot1.base = SE3(x,y,z))
        # Add robots to the Swift environment: self.env.add(self.robot1)
        
        # self.env.add(self.robot_test)
        self.robot2.add_to_env(self.env)
        self.robot2.base = SE3(5.8486, 6.4944, 0.5)

        self.robot1.add_to_env(self.env)
        self.robot1.base = SE3(9.72,5.6,0.5)

        self.robot3.add_to_env(self.env)
        self.robot3.base = SE3(4.6,4.05,1.0)
        
        # TODO: Create conveyor belts and add to scene
        # e.g., conv1 = ConveyorBelt(start=SE3(...), end=SE3(...), speed=0.1)
        # self.conveyors.append(conv1)
        # (You might represent conveyors visually by adding a mesh or using shapes)
        #
        # TODO: Load the motorbike model or shape and add to scene
        # e.g., bike_mesh = geometry.Mesh('motorbike.stl', pose=SE3(x,y,z), color=(...))
        # self.motorbike = bike_mesh
        # self.env.add(self.motorbike)
        #
        # TODO: Add any safety barriers or light curtain visuals (if any)
        # e.g., a transparent plane or lines indicating the light curtain region.
    
    def add_object(self, obj):
        """Add a manipulatable object (pizza, topping, etc.) to the environment and keep track of it."""
        # Add object to environment and internal list
        if self.env:
            self.env.add(obj.mesh)  # assuming obj has a .mesh attribute for its geometry
        self.objects.append(obj)
    
    def remove_object(self, obj):
        """Remove an object from the environment and internal tracking (e.g., when pizza leaves on bike)."""
        if self.env:
            try:
                self.env.remove(obj.mesh)
            except Exception:
                pass  # If the environment doesn't support remove, we may just hide or ignore
        if obj in self.objects:
            self.objects.remove(obj)

class ConveyorBelt:
    """Represents a conveyor belt that moves objects from a start position to an end position."""
    def __init__(self, start_pose: SE3, end_pose: SE3, speed: float):
        self.start_pose = start_pose  # SE3 start location of conveyor
        self.end_pose = end_pose      # SE3 end location of conveyor
        self.speed = speed           # movement speed (units per second) along conveyor
        self.active = True           # whether the conveyor is currently moving or stopped
    
    def move_object(self, obj: ManipulatableObject, dt: float) -> bool:
        """Move the given object along the conveyor by distance = speed*dt.
        Returns True if object has reached the end of conveyor."""
        # TODO: Compute movement along conveyor direction.
        # You might parameterize conveyor by a vector from start to end and move object accordingly.
        # For simplicity, assume straight line from start_pose to end_pose.
        # Update obj position by small step towards end.
        # If end reached or exceeded, return True (meaning object should transfer to next stage or conveyor).
        return False
