from manipulatable_object import ManipulatableObject

import swift
import roboticstoolbox as rtb
from ir_support import LinearUR3
from spatialmath import SE3
from spatialmath.base import *
from math import pi

class World:
    """Simulation world: handles environment launch, and loading of robots, objects, and safety elements."""
    def __init__(self):
        self.env = swift.Swift()   # The Swift environment (or other simulator) instance
        self.robot1 = None         # Robot performing sauce application
        self.robot2 = None         # Robot performing topping placement
        self.robot3 = None         # Robot handling oven loading/unloading
        self.robot4 = None         # Robot packaging and loading pizza on bike
        self.conveyors = []        # List of ConveyorBelt objects in the system
        self.motorbike = None      # The motorbike object (for delivery)
        self.safety_barriers = []  # Any safety barrier objects (e.g., fences, light curtain representation)
        self.objects = []          # List to hold other objects (pizzas, toppings, etc. currently in scene)
        # (Other environment attributes can be added as needed)
    
    def launch(self):
        """Start the simulator and set up the static scene (floor, walls, etc.)."""
        # TODO: Create and launch the Swift environment
        # Example:
        # self.env = swift.Swift()
        # self.env.launch(realtime=True)
        # Add floor or background:
        # floor = geometry.Box(..., pose=SE3(0,0,-0.01), color=(0.5,0.5,0.5,1))
        # self.env.add(floor)
        pass
    
    def setup_robots_and_objects(self):
        """Load robots, conveyors, and objects into the environment."""
        # TODO: Initialize robot models (from RTB or custom DH parameters)
        # e.g., self.robot1 = rtb.models.DH.SomeRobotModel() or custom Robot class
        # Position robots in the scene (adjust base if needed, e.g., robot1.base = SE3(x,y,z))
        # Add robots to the Swift environment: self.env.add(self.robot1)
        #
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
        pass
    
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
