import external_e_stop
import flask_GUI
from logger import Logger
import manipulatable_object
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement
from run_sim import Run
from states import States
import step_environment
from world import World

def main():
    """Main entry point: Initialize world, robots, GUI, and run the simulation."""
    # Initialize simulation world and environment
    world = World()
    world.launch()  # Launch Swift simulator and load environment objects
    
    # Initialize robots and other objects in the world
    # (Assume World class handles adding robots, conveyors, etc.)
    world.setup_robots_and_objects()
    
    # Initialize GUI (Flask app) and safety systems
    gui = flask_GUI()
    estop = external_e_stop()
    logger = Logger()
    state = States(world)  # pass world or needed references to track state
    
    # Initialize motion controllers for each robot, passing in robot models from world
    motion1 = Robot1Movement(world.robot1)
    motion2 = Robot2Movement(world.robot2)
    motion3 = Robot3Movement(world.robot3)
    motion4 = Robot4Movement(world.robot4)
    
    # Create the main runner and pass all components
    runner = Run(world, gui, estop, state, logger, [motion1, motion2, motion3, motion4])
    # Optionally, start the GUI in a separate thread so it runs concurrently
    gui.start_server_async()  # This would start the Flask app without blocking the main thread
    
    # Start the main simulation loop (this will run until stopped)
    runner.run_loop()
    
    # After loop ends (e.g., simulation stopped), hold the environment open
    world.env.hold()  # Keep Swift window open (if needed for viewing after stop)
    
    
if __name__ == '__main__':
    main()