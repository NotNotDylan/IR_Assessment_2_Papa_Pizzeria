import external_e_stop
from imgui_GUI import GUIImGui
from logger import Logger
import manipulatable_object
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement
from run_sim import Run
from states import States
import step_environment
from world import World

from ir_support import DHRobot3D

def main():
    """Main entry point: Initialize world, robots, GUI, and run the simulation."""
    # Initialize simulation world and environment
    world = World()
    world.launch()  # Launch Swift simulator and load environment objects
    
    # Initialize robots and other objects in the world
    # (Assume World class handles adding robots, conveyors, etc.)
    world.setup_robots_and_objects()
    
    # Initialize GUI and safety systems
    gui = GUIImGui()
    estop = external_e_stop()
    logger = Logger()
    state = States(world)  # pass world or needed references to track state
    
    # Bind robots to GUI via small adapters
    def adapter(robot_model: DHRobot3D):
        return {
            'name': robot_model.__class__.__name__,
            'get_q': lambda: list(robot_model.q),
            'set_q': lambda q: setattr(robot_model, 'q', q),   # optional direct apply
            'qlim': list(robot_model.qlim) if hasattr(robot_model, 'qlim') else [(-3.14, 3.14)] * len(robot_model.q),
            'dof': len(robot_model.q),
        }

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
    runner = Run(world=world, gui=None, estop=estop, state=state, logger=logger, motions=motions)
    
    # Start the main simulation loop (this will run until stopped)
    runner.run_loop()
    
    # After loop ends (e.g., simulation stopped), hold the environment open
    # world.env.hold()  # Keep Swift window open (if needed for viewing after stop)
    
    
if __name__ == '__main__':
    main()