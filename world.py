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
import numpy as np
import threading



class World:
    """Simulation world: handles environment launch, and loading of robots, objects, and safety elements."""
    def __init__(self):
        self.env = swift.Swift()   # The Swift environment instance
        self.robot_test = None   # Robot I am using to temporaly test the GUI
        self.robot1 = UR3()         # Robot performing sauce application
        self.robot2 = None        # Robot performing topping placement
        self.robot3 = IRB_4600()        # Robot handling oven loading/unloading
        self.robot4 = IRB2400()        # Robot packaging and loading pizza on bike
        self.conveyors = []        # List of ConveyorBelt objects in the system
        self.motorbike = None      # The motorbike object (for delivery)
        self.safety_barriers = []  # Any safety barrier objects (e.g., fences, light curtain representation)
        self.objects = []          # List to hold other objects (pizzas, toppings, etc. currently in scene)
        # (Other environment attributes can be added as needed)
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

            Conveyer_One = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "First_Conveyer2.0.3.stl"),
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
            
            self.plate = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
                          pose = SE3(self.x ,self.y ,self.z), color=(0.25,0.25,0.25,1.0),scale=[1, 1, 1])
            self.env.add(self.plate)

            self.plate2 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
                          pose = SE3(self.x+0.1 ,self.y ,self.z), color=(0.35,0.35,0.35,1.0),scale=[1, 1, 1])
            self.env.add(self.plate2)

            self.pizza = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Base.stl"), 
                          pose = SE3(self.x_pizza, self.y_pizza, self.z_pizza), color=(0.90, 0.83, 0.70))
            self.env.add(self.pizza)

            self.sauce = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Sauce.stl"), 
                              pose = SE3(0,0,0), color=(0.698, 0.133, 0.133))



            

            print(self.plates)
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
        self.robot1.base = SE3(4.6,4.05,1.0)
        self.robot1.add_to_env(self.env)
        
        self.robot3.base = SE3(9.72,5.6,0.5)
        self.robot3.add_to_env(self.env)

        self.robot4.base = SE3(5.84, 6.49, 0.5)
        self.robot4.add_to_env(self.env)
        
        
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

    # def conveyorBelt_Movement(self):
    #     self.env.swift_objects.remove(self.plate)
    # def safe_remove_from_swift(self, env, obj):
    #     if not hasattr(env, "swift_objects"):
    #         return
    #     swift_objects = getattr(env, "swift_objects")

    #     # Iterate through the list in steps of 2 because it’s [obj, pose, obj, pose, ...]
    #     for i in range(0, len(swift_objects) - 1, 2):
    #         if swift_objects[i] is obj:
    #             # Remove both the object and its associated pose
    #             del swift_objects[i:i+2]
    #             print(f"Removed {obj} safely from Swift environment")
    #             return
    #     print(f"Object {obj} not found in Swift environment")


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
            print(self.pizza.T) 
            #if (self.pizza.T) != (SE3(x1, y1, z1).A):
            if self.t - self.last_time_pizza >= period:
                self.pizza.T = self.pizza.T @ SE3(self.displacement, 0, 0).A
                self.sauce_movement()
                self.last_time_pizza = self.t
            self.conveyorBelt_Movement(plate1=self.plate,plate2=self.plate2,direction=1,period=0.25)
            
    
    def pizza_timing(self, pause_1, pause_2):
        self.t = float(self.env.sim_time)
        match self.pos:
            case 1:
                self.t = float(self.env.sim_time)
                self.last_pizza_time = float(self.env.sim_time) 
                self.pizza_movement(pos=self.pos1,period=0.25)
                if np.allclose((self.pizza.T), (SE3(self.pos1).A)):
                    self.pos = 2

            case 2:
                self.t = float(self.env.sim_time)
                if self.t - self.last_pizza_time >= pause_1:
                    self.pizza_movement(pos=self.pos2,period=0.25)
                if np.allclose((self.pizza.T), (SE3(self.pos2).A)):
                    self.last_pizza_time = float(self.env.sim_time)
                    self.pos = 3
                    
            case 3:
                self.t = float(self.env.sim_time)
                if self.t - self.last_pizza_time >= pause_2:
                    self.pizza_movement(pos=self.pos3,period=0.25)

    def sauce_placement(self):
        if np.allclose((self.pizza.T), (SE3(self.pos1).A)):
            self.sauce.T = self.pizza.T @ SE3(0,0,0.0075).A
            self.env.add(self.sauce)
            self.sauce_placed = True

    def sauce_movement(self):
        if self.sauce_placed == True:
            self.sauce.T = self.pizza.T @ SE3(0,0,0.0075).A
        
            

            



                
                
            





        

        



    

        


