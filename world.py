from manipulatable_object import ObjectNode
import swift
import roboticstoolbox as rtb
from ir_support import UR3
from ABB_IRB_2400.IRB_2400 import IRB2400
from IRB_4600.ABB_IRB_4600 import IRB_4600
from AuboI5.i5Init import AuboI5
from spatialmath import SE3
from spatialmath.base import *
from math import pi
import time 
from spatialgeometry import Sphere, Arrow, Mesh, Cuboid, Cylinder
import spatialgeometry as geometry
import os
import spatialmath.base as spb
from spatialmath import SE3
import numpy as np
import threading
from ir_support import RectangularPrism, line_plane_intersection, CylindricalDHRobotPlot



class World:
    """Simulation world: handles environment launch, and loading of robots, objects, and safety elements."""
    def __init__(self):
        self.env = swift.Swift()   # The Swift environment instance
        self.robot1 = UR3()        # Robot performing sauce application
        self.robot2 = AuboI5()     # Robot performing topping placement
        self.robot3 = IRB_4600()   # Robot handling oven loading/unloading
        self.robot4 = IRB2400()    # Robot packaging and loading pizza on bike
        self.conveyors = []        # List of ConveyorBelt objects in the system
        self.motorbike = None      # The motorbike object (for delivery)
        self.safety_barriers = []  # Any safety barrier objects (e.g., fences, light curtain representation)
        self.objects = []          # List to hold other objects (pizzas, toppings, etc. currently in scene)
        # (Other environment attributes can be added as needed)
        self.collision_opacity = 0.001 # Can't be 0 or it will crash
               
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
        self.pos3 = SE3(8.8, 3.6, 1.015)
        self.pos = 1
        self.sauce_placed = False
        self.xp = 7.5
        self.yp = 6.5
        self.meshes = []
        

    
    def launch(self, environment_objects: bool = False):
        """Start the simulator and set up the static scene (floor, walls, etc.)."""
      
        self.env.launch(realtime=True)
        
        self.env.set_camera_pose([8, 14, 7], [8, 3, 0])
        
        # --Important meshes--
        Table = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Table.stl"),
                    color=(0.588, 0.294, 0.0))
        self.env.add(Table)
        
        Pizza_Oven = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pizza_Oven2.stl"),
                    color=(0.886,0.447,0.357,1.0))
        self.env.add(Pizza_Oven)
        
        Toppings_Table = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Table_Toppings7.stl"),
                    color=(0.5,0.5,0.5,1.0), pose=SE3(0, 0, 0))
        self.env.add(Toppings_Table)
        
        self.plate1 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
                        pose = SE3(self.x ,self.y ,self.z), color=(0.25,0.25,0.25,1.0),scale=[1, 1, 1])
        self.env.add(self.plate1)

        self.plate2 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
                        pose = SE3(self.x+0.1 ,self.y ,self.z), color=(0.35,0.35,0.35,1.0),scale=[1, 1, 1])
        self.env.add(self.plate2)
        
        
        if environment_objects == True:
            # Walls
            Conveyer_One = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "First_Conveyer2.0.3.stl"),
                       color=(0.5,0.5,0.5,1.0))
            self.env.add(Conveyer_One)

            # Table = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Table.stl"),
            #             color=(0.588, 0.294, 0.0))
            # self.env.add(Table)
            
            # Pizza_Oven = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Pizza_Oven2.stl"),
            #             color=(0.886,0.447,0.357,1.0))
            # self.env.add(Pizza_Oven)

            Light_Fence_Post = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Light_Fence_Post.stl"),
                        color=(0.1,0.1,0.1,1.0))
            self.env.add(Light_Fence_Post)
            
            

            # Toppings_Table = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Table_Toppings7.stl"),
            #             color=(0.5,0.5,0.5,1.0), pose=SE3(0, 0, 0))
            # self.env.add(Toppings_Table)
            
            # self.plate1 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
            #               pose = SE3(self.x ,self.y ,self.z), color=(0.25,0.25,0.25,1.0),scale=[1, 1, 1])
            # self.env.add(self.plate1)

            # self.plate2 = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Conveyor_Movement.stl"), 
            #               pose = SE3(self.x+0.1 ,self.y ,self.z), color=(0.35,0.35,0.35,1.0),scale=[1, 1, 1])
            # self.env.add(self.plate2)



            #UNNEEDED
            wall = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Wall.stl"),
                        color=(1.0,1.0,1.0,1.0))
            self.env.add(wall)

            floor = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Floor.stl"),
                        color=(1.0,1.0,0.0,1.0))
            self.env.add(floor)

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
            
            # Pizza
            self.pizza = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Base.stl"), 
                          #pose = SE3(self.x_pizza, self.y_pizza, self.z_pizza), 
                          #pose = SE3(7.5, 6.5, 0.475905*2),
                          pose = SE3(self.pos3),
                          color=(0.90, 0.83, 0.70))
            self.env.add(self.pizza)

            self.sauce = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Sauce.stl"), 
                              #pose = SE3(7.5,6.5,(0.475905*2)+0.0075),
                              pose = SE3(self.xp, self.yp, (0.475905*2)+0.0075), 
                              color=(0.698, 0.133, 0.133))
            self.env.add(self.sauce)

            # self.cheese = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Cheese.stl"),
            #                    pose = SE3(self.xp, self.yp, 1.0125),
            #                    color=(1.0, 0.78, 0.24))
            
            self.cheese_pile = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Cheese_Pile.stl"), 
                                    pose = SE3(6.5,4,1.004),
                                    color=(1.0, 0.78, 0.24))
            self.env.add(self.cheese_pile)


            # self.olives = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Olives.stl"), 
            #                   #pose = SE3(7.5,6.5,(0.475905*2)+0.0075),
            #                   pose = SE3(self.xp, self.yp, (0.475905*2) +0.0125), 
            #                   color=(0.20, 0.20, 0.20))
            
            self.olive_pile = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Olives_Pile.stl"), 
                                    pose = SE3(6.2,4,1),
                                    color=(0.20, 0.20, 0.20))
            self.env.add(self.olive_pile)
            

            # self.ham = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Ham.stl"), 
            #                   #pose = SE3(7.5,6.5,(0.475905*2)+0.0075),
            #                   pose = SE3(self.xp, self.yp, (0.475905*2)+0.0125), 
            #                   color=(1.0, 0.71, 0.76))
            self.ham_pile = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pepperoni_Pileblend.stl"), 
                                    pose = SE3(5.9,4,1),
                                    color=(1.0, 0.71, 0.76))
            self.env.add(self.ham_pile)
            

            # self.pepperoni = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pepperoni.stl"), 
            #                   #pose = SE3(7.5,6.5,(0.475905*2)+0.0075),
            #                   pose = SE3(self.xp, self.yp, (0.475905*2)+0.0125), 
            #                   color=(0.71, 0.20, 0.14))
            self.pepperoni_pile = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pepperoni_Pileblend.stl"), 
                                    pose = SE3(6.8,4,1),
                                    color=(0.71, 0.20, 0.14))
            self.env.add(self.pepperoni_pile)
            
            

            # self.pineapple = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pineapple.stl"), 
            #                   #pose = SE3(7.5,6.5,(0.475905*2)+0.0075),
            #                   pose = SE3(self.xp, self.yp, (0.475905*2)+0.0125), 
            #                   color=(1.0, 0.90, 0.39))
            self.pineapple_pile = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pineapple_Pizza.stl"), 
                                    pose = SE3(7.1,4,1.005),
                                    color=(1.0, 0.90, 0.39))
            self.env.add(self.pineapple_pile)
            
            
            self.melted_cheese = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Melted_Cheese.stl"), 
                              pose = SE3(self.xp, self.yp ,(0.475905*2)+0.01241), color=(1.0, 0.78, 0.25))
            

            self.box = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Box2.stl"),
                                pose = SE3(self.xp, self.yp, 1), 
                                color=(1.0, 1.0, 1.0))
            self.env.add(self.box)

            self.motorbike = Mesh(filename=os.path.join(os.path.dirname(__file__), "Environment", "Honda Hornet STL.stl"),
                                pose = SE3(3, 8.0, 0.0).A @ spb.trotz(pi),
                                color=(0.8,0.8,0.8))
            self.env.add(self.motorbike)

            # self.pizza = Mesh(filename=os.path.join(os.path.dirname(__file__), "Pizza's", "Pizza_Base.stl"), 
            #               pose = SE3(self.x_pizza, self.y_pizza, self.z_pizza), color=(0.90, 0.83, 0.70))
            # self.env.add(self.pizza)


            # converyer_collision1 = Cuboid(scale=[1, 1.2, 1.5], base=SE3(3.5, 3.6, 0.75), color=(1.0,1.0,1.0,self.collision_opacity))
            # self.env.add(converyer_collision1)

            # converyer_collision2 = Cuboid(scale=[5, 0.4, 1], base=SE3(6.5, 3.6, 0.5), color=(1.0,1.0,1.0,self.collision_opacity))
            # self.env.add(converyer_collision2)

            # table_collision = Cuboid(scale=[1, 1.5, 1], base=SE3(7.5, 6.4935, 0.5), color=(1,1,1,self.collision_opacity))
            # self.env.add(table_collision)
                        
            # pizza_oven_collision = Cuboid(scale=[3,3,1], base=SE3(11.983, 7.8631, 0.5), color=(1,1,1,self.collision_opacity))
            # self.env.add(pizza_oven_collision)

            light_fence_collision = Cuboid(scale=[4.22, 0.05, 3], base=SE3(5.21, 4.75, 1.5), color=(1,1,1,0.5))
            self.env.add(light_fence_collision)

            # pillar1_collision = Cuboid(scale=[0.4,0.4,0.5], base=SE3(5.8486, 6.4944, 0.25), color=(1,1,1,self.collision_opacity))
            # self.env.add(pillar1_collision)

            # pillar2_collision = Cuboid(scale=[0.4,0.4,0.5], base=SE3(9.72, 5.6, 0.25), color=(1,1,1,self.collision_opacity))
            # self.env.add(pillar2_collision)

            # pillar3_collision = Cuboid(scale=[0.4,0.4,1], base=SE3(4.6, 4.05, 0.5), color=(1,1,1,self.collision_opacity))
            # self.env.add(pillar3_collision)

            # pillar4_collision = Cuboid(scale=[0.4,0.4,1], base=SE3(6.52, 4.4, 0.5), color=(1,1,1,self.collision_opacity))
            # self.env.add(pillar4_collision)

            # toppings_collision = Cuboid(scale=[1.5,0.3,1], base=SE3(6.5, 4, 0.5), color=(1,1,1,self.collision_opacity))
            # self.env.add(toppings_collision)

        # self.bounds_array = np.array([
        #     [3.0, 4.0, 3.0, 4.2, 0.0, 1.5],
        #     [4.0, 9.0, 3.4, 3.8, 0.0, 1.0],
        #     [7.0, 8.0, 5.7435, 7.2435, -0.024095, 0.975905],
        #     [10.483, 13.483, 6.3631, 9.3631, 0.0, 1.0],
        #     [3.10, 7.32, 4.725, 4.775, 0.0, 3.0],
        #     [5.6486, 6.0486, 6.2944, 6.6944, 0.0, 0.5],
        #     [9.52, 9.92, 5.4, 5.8, 0.0, 0.5],
        #     [4.4, 4.8, 3.85, 4.25, 0.0, 1.0],
        #     [6.32, 6.72, 4.2, 4.6, 0.0, 1.0],
        #     [6.0, 7.0, 3.875, 4.125, 0.0, 1.0]
        # ])
        #
        # print(self.bounds_array[0][0])
    
    def setup_robots_and_objects(self):
        """Load robots, conveyors, and objects into the environment."""
        
        # self.env.add(self.robot_test)
        self.robot1.base = SE3(4.6,4.05,1.0) @ SE3.Rz(pi)
        self.robot1.add_to_env(self.env)

        self.robot2.base = SE3(6.52,4.4,1.0)
        self.robot2.add_to_env(self.env)
        
        T = self.robot2.fkine(self.robot2.q) 
        print(T)

        
        self.robot3.base = SE3(9.72,5.6,0.5)
        self.robot3.add_to_env(self.env)

        self.robot4.base = SE3(5.84, 6.49, 0.5)
        self.robot4.add_to_env(self.env)
    
    '''def add_object(self, obj):
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
            self.sauce.T = self.pizza.T @ SE3(0,0,0.0075).A'''
