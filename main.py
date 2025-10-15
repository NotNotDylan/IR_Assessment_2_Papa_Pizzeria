import external_e_stop
from imgui_GUI import GUIImGui
from logger import Logger
import manipulatable_object
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement
from run_sim import Run
from states import State
from world import World

from ir_support import DHRobot3D
import numpy as np


def adapter(robot_model: DHRobot3D):
    '''Given a robot this function will extract the preameters the GUI requires to controle it'''

    qlim_pairs = [(-2*np.pi, 2*np.pi)] * len(robot_model.q)  # fallback
    qlim_arr = getattr(robot_model, 'qlim', None)

    # Ensures thevalues are in the correct shape as to not trip up the GUI
    if qlim_arr is not None:
        qa = np.asarray(qlim_arr, dtype=float)
        if qa.ndim == 2:
            if qa.shape[0] == 2:          # 2 x N  → transpose to N x 2
                qlim_pairs = [tuple(row) for row in qa.T]
            elif qa.shape[1] == 2:        # N x 2
                qlim_pairs = [tuple(row) for row in qa]
        # else leave fallback

    return {
        'name': robot_model.__class__.__name__,
        'get_q': lambda: list(robot_model.q),
        'set_q': lambda q: setattr(robot_model, 'q', q),
        'qlim': qlim_pairs,
        'dof' : len(robot_model.q),
    }

def main():
    """Main entry point: Initialize world, robots, GUI, and run the simulation."""
    # Initialize simulation world and environment
    world = World()
    world.launch(environment_objects=False)  # Launch Swift simulator and load environment objects
    # Initialize robots and other objects in the world
    # (Assume World class handles adding robots, conveyors, etc.)
    world.setup_robots_and_objects()
    
    # Initialize GUI and safety systems
    gui = GUIImGui()
    # estop = external_e_stop()
    # logger = Logger()
    

    # robots_dict = {
    #     1: adapter(world.robot1),
    #     2: adapter(world.robot2),
    #     3: adapter(world.robot3),
    #     4: adapter(world.robot4),
    # }
    
    robots_dict = {  #TODO: Swap back this is tempory to test loading in and moving a robot
        1: adapter(world.robot_test)
    }
    
    # Begin GUI
    gui.bind_robots(robots_dict, default_robot=1)
    gui.start()

    # Initialize motion controllers for each robot, passing in robot models from world
    motions = [Robot1Movement(world.robot1), 
               Robot2Movement(world.robot2),
               Robot3Movement(world.robot3), 
               Robot4Movement(world.robot4)]
    
    # Create the main runner and pass all components
    runner = Run(world=world, gui=gui, estop=None, logger=None, motions=motions)
    
    # Start the main simulation loop (this will run until stopped)
    runner.run_loop()
    
    # After loop ends (e.g., simulation stopped), hold the environment open
    # world.env.hold()  # Keep Swift window open (if needed for viewing after stop)
    
    
if __name__ == '__main__':
    main()