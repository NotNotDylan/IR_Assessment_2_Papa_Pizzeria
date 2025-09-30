import spatialgeometry as geometry
from spatialmath import SE3

class ManipulatableObject:
    """Base class for objects that can be manipulated by robots (pizza bases, toppings, etc.)."""
    def __init__(self, name="Object", init_pose=None):
        self.name = name
        # Use a simple shape if no mesh available
        # For example, a cylinder for pizza, small spheres or cylinders for toppings.
        # Subclasses can override the geometry.
        if init_pose is None:
            init_pose = SE3(0, 0, 0)
        # Default: unit sphere as placeholder
        self.mesh = geometry.Sphere(radius=0.02, pose=init_pose, color=(1,0,0,1))  # red sphere as default
        self.pose = init_pose  # store pose as SE3 for convenience
        self.id = None

class Pizza(ManipulatableObject):
    """Pizza base object, which can have toppings and be moved by conveyors."""
    def __init__(self, id, init_pose=None):
        super().__init__(name=f"Pizza{id}", init_pose=init_pose)
        # Override geometry to a flat cylinder representing a pizza base
        # e.g., radius 0.15m, height 0.02m, color brown
        self.mesh = geometry.Cylinder(radius=0.15, length=0.02, pose=init_pose, color=(0.8, 0.6, 0.4, 1))
        self.id = id
        self.toppings = []     # list of Topping objects on this pizza
        self.on_conveyor = None  # which conveyor it's currently on (assigned in States when spawned)
        self.stage = "spawned"  # stage in process ("spawned", "ready_for_sauce", "sauce_done", etc.)
        self.in_box = False    # whether pizza is in a box
        self.delivered = False # whether pizza has been delivered (loaded on bike)

class Topping(ManipulatableObject):
    """Topping object that can be picked and placed onto pizzas."""
    def __init__(self, type="Topping", init_pose=None):
        super().__init__(name=type, init_pose=init_pose)
        # Optionally override geometry based on type (e.g., different color or shape for different toppings)
        self.type = type
        # Example: if type is "pepperoni", make small red flat cylinder
        if type.lower() == "pepperoni":
            self.mesh = geometry.Cylinder(radius=0.03, length=0.005, pose=init_pose, color=(0.9, 0.2, 0.2, 1))
        elif type.lower() == "mushroom":
            self.mesh = geometry.Sphere(radius=0.02, pose=init_pose, color=(0.8, 0.8, 0.7, 1))
        else:
            # default small cube
            self.mesh = geometry.Box(scale=(0.02, 0.02, 0.01), pose=init_pose, color=(0.5, 0.5, 0.5, 1))
        self.attached_to_pizza = None
        self.relative_pose_on_pizza = None
