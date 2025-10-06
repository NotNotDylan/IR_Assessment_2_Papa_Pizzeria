import external_e_stop
import imgui_GUI #TODO: Change all of the Flask_GUI functions to imgui functions
from logger import Logger
import manipulatable_object
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement
from states import States
import step_environment
from world import World

import time
import threading
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
# ... (other imports like numpy, logging, flask etc., as needed)

class Run:
    """Main controller for running the simulation loop and coordinating all components."""
    def __init__(self, world: World, gui: imgui_GUI, estop: external_e_stop, 
                 state: States, logger: Logger, motions: list):
        self.world = world
        self.gui = gui
        self.estop = estop
        self.state = state
        self.logger = logger
        self.motions = motions  # list of robot motion controller objects
        self.running = True
        self.paused = False  # indicates if simulation is paused (e.g., after e-stop)
    
    def run_loop(self):
        """Run the main simulation loop, updating state, handling inputs, and moving robots."""
        # Simulation loop runs until `self.running` is False (could be set by GUI or other stop condition)
        while self.running:
            # 1. Check for emergency stop signals (hardware or GUI e-stop)
            if self.estop.check_stop() or self.gui.estop_pressed:
                # If an e-stop is triggered, log it and enter a paused state
                self.logger.log_event("Emergency stop triggered! Pausing all operations.")
                self.paused = True
                # Optionally, command all robots to stop immediately (e.g., set zero velocities)
                for motion in self.motions:
                    motion.emergency_stop()
            
            # 2. If paused (either due to e-stop or user pause), wait until resume is requested
            if self.paused:
                # If e-stop is released and user hits resume on GUI, then continue
                if self.gui.resume_requested and not self.estop.is_pressed:
                    self.logger.log_event("Resuming operations after pause.")
                    self.paused = False
                    self.gui.resume_requested = False
                else:
                    # Skip the rest of loop and continue checking stop/resume
                    time.sleep(0.01)
                    self.world.env.step(0.01)  # still update environment to keep it responsive
                    continue  # go back to start of loop
            
            # 3. Update the state of pizzas, conveyors, etc. (e.g., if a pizza moved to next station)
            self.state.update()
            
            # 4. Compute movements for each robot based on current state and task
            for motion in self.motions:
                # Each robot's motion controller decides its next action or continues current one
                motion.update(self.state)
            
            # 5. Log the current status (optional: log robot joint angles, pizza positions, etc.)
            self.logger.log_status(self.state, self.motions)
            
            # 6. Apply the calculated movements and environment updates (move robots, conveyors, spawn/despawn objects)
            step_environment.apply(self.world, self.state, self.motions)
            
            # 7. Step the simulation environment forward by a time increment (e.g., delta_t)
            self.world.env.step(self.state.time_step)
            
            # 8. (Optional) Check for a condition to stop the simulation loop
            if self.gui.stop_pressed:
                self.logger.log_event("Stop button pressed. Exiting simulation loop.")
                self.running = False
        
        # End of loop - clean up if necessary (e.g., close env if not holding)
        # (The world.env.hold() call can be done after this function returns.)
