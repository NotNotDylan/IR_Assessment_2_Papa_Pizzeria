from world import World
from manipulatable_object import Pizza

class States:
    """Tracks the state of pizzas, robots, and sensors in the system."""
    def __init__(self, world: World):
        self.world = world
        self.pizzas = []        # list of Pizza objects currently in system
        self.current_id = 0     # ID counter for pizzas for identification
        self.light_curtain_broken = False  # True if something triggered the safety light curtain
        # Robot busy/free status (could be tracked via robot motion classes too)
        self.robot_busy = {1: False, 2: False, 3: False, 4: False}
        # Conveyor states if needed (e.g., active or not)
        # You can derive from world.conveyors list
        self.time_step = 0.05   # simulation time step (for use in motions and environment stepping)
    
    def spawn_pizza(self):
        """Spawn a new pizza at the beginning of the line (on the first conveyor)."""
        # Create a new Pizza object at the start of conveyor 1
        pizza = Pizza(id=self.current_id, init_pose=self.world.conveyors[0].start_pose)
        self.current_id += 1
        self.pizzas.append(pizza)
        self.world.add_object(pizza)
        self.robot_busy[1] = False  # Robot1 should be free to handle this new pizza
        return pizza
    
    def update(self):
        """Update system state each cycle: move conveyors, check positions, transitions, and sensors."""
        # Move pizzas along conveyors if active
        for conv in self.world.conveyors:
            if conv.active:
                for pizza in list(self.pizzas):  # use list copy since we may remove
                    if pizza.on_conveyor == conv:
                        reached_end = conv.move_object(pizza, self.time_step)
                        if reached_end:
                            # Pizza has reached end of this conveyor
                            # Determine next stage:
                            conv_index = self.world.conveyors.index(conv)
                            if conv_index < len(self.world.conveyors) - 1:
                                # Move pizza to next conveyor
                                next_conv = self.world.conveyors[conv_index + 1]
                                pizza.on_conveyor = next_conv
                                # Possibly position pizza at start of next conveyor
                                pizza.mesh.pose = next_conv.start_pose
                            else:
                                # Last conveyor end (pizza reached packaging area / Robot4)
                                pizza.on_conveyor = None  # off conveyors now
                            # Stop this conveyor if needed (e.g., if only one pizza at a time per conveyor station)
                            conv.active = False
        
        # Check if a new pizza needs to be spawned (for continuous operation)
        # e.g., if no active pizza on first conveyor and Robot1 is free, spawn another
        if all(p.on_conveyor != self.world.conveyors[0] for p in self.pizzas):
            # If first conveyor is free, spawn a new pizza onto it
            self.spawn_pizza()
        
        # Check pizza positions to update which robot should act
        for pizza in self.pizzas:
            # Example condition: if pizza is on conveyor 1 but conveyor 1 is inactive (meaning pizza stopped at Robot1 position)
            # and Robot1 is free -> Robot1 can start sauce task.
            if pizza.on_conveyor == self.world.conveyors[0] and not self.world.conveyors[0].active:
                if not self.robot_busy[1]:
                    pizza.stage = "ready_for_sauce"
                    # Robot1 will pick this up in its update() by seeing pizza.stage
                    # Mark robot1 busy now (to avoid multiple triggers)
                    self.robot_busy[1] = True
            # Similarly, check for robot2 stage (pizza moved to conv2 and conv2 stopped, etc.)
            if pizza.on_conveyor == self.world.conveyors[1] and not self.world.conveyors[1].active:
                if not self.robot_busy[2]:
                    pizza.stage = "ready_for_toppings"
                    self.robot_busy[2] = True
            # Check for robot3 (oven) stage: if pizza reached conv3 end
            if pizza.on_conveyor == self.world.conveyors[2] and not self.world.conveyors[2].active:
                if not self.robot_busy[3]:
                    pizza.stage = "ready_for_oven"
                    self.robot_busy[3] = True
            # Check for robot4 (packing) stage: if pizza off conveyors (i.e., after oven output) and not handled yet
            if pizza.on_conveyor is None and pizza.stage == "out_of_oven":
                if not self.robot_busy[4]:
                    pizza.stage = "ready_for_pack"
                    self.robot_busy[4] = True
        
        # Safety sensor: Light curtain check
        # Define the zone that constitutes the light curtain (for example, a vertical plane at a certain X position)
        # Here we assume if any object (pizza or otherwise) goes beyond X = some value or in a restricted area, we trigger.
        for obj in self.world.objects:
            # Suppose our light curtain is at x = 0.5 (just as an example threshold) and covers certain y,z range.
            x = obj.mesh.pose.t[0]  # assuming spatialmath SE3, .t gives translation vector
            if x > 0.5:  # if object crosses beyond this X (meaning out of safe zone)
                self.light_curtain_broken = True
                break
        else:
            self.light_curtain_broken = False
