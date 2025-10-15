import external_e_stop
from imgui_GUI import GUIImGui
from logger import Logger
import manipulatable_object
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement, MovementCalculation
from states import State
from world import World
from states import State
from states import SystemState as SS

import time
import threading
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
# ... (other imports like numpy, logging, flask etc., as needed)

class Run:
    """Main controller for running the simulation loop and coordinating all components."""
    def __init__(self, world: World, gui: GUIImGui, estop: external_e_stop, 
                 logger: Logger, motions: list):
        self.world = world
        self.gui = gui
        self.external_estop = estop
        self.logger = logger
        self.motions = motions  # list of robot motion controller objects
        self.running = True
        self.paused = False  # indicates if simulation is paused (e.g., after e-stop)
        
        # Creating only one instance of the robot movment and calcuation objects
        self.robot_test_motion = MovementCalculation(self.world.robot_test)
        # self.robot1_motion = self.motions[0]
        # self.robot2_motion = self.motions[1]
        # self.robot3_motion = self.motions[2]
        # self.robot4_motion = self.motions[3]
        
        self.dt = 0.05 # change in time between updates
        self.time_in_loop = 0
        # self.counter = 0
        
    
    def run_loop(self):
        """Run the main simulation loop, updating state, handling inputs, and moving robots."""
        # Simulation loop runs until `self.running` is False (could be set by GUI or other stop condition)
        qtraj = self.test_inverse_kinamatics()
        
        inputs = {
            "Test qtraj" : qtraj
        }
        
        # Initilises all the varable states used in the program
        self.state_init()
        
        while self.running:
            # For event managment
            self.time_in_loop = float(self.world.env.sim_time)
            
            # Update states
            self.handle_states()
            
            # Update GUI and process events  
            self.handle_gui()
            
            # Effects are enacted
            self.handle_actions(inputs)
            
            # if self.counter < len(qtraj):
            #     self.world.robot_test.q = qtraj[self.counter]
            #     self.world.env.step(self.dt)
                
            # Step the environment to update visuals
            self.world.env.step(self.dt)
            
            # self.counter += 1
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(0.01)
            
            ''' Structure I can follow:
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
                '''
        
        # End of loop - clean up if necessary (e.g., close env if not holding)
        # (The world.env.hold() call can be done after this function returns.)
    
    def test_inverse_kinamatics(self):
        curent_q = self.world.robot_test.q
        
        goal = SE3(0.5788,1.27,0.8437)
        goal_q = self.robot_test_motion.inverse_kinematics(goal)
        
        qtraj = rtb.jtraj(curent_q, goal_q, 80).q
        return qtraj
        
    def set_robot_to_move_GUIHelper(self, robot_id):
        if robot_id == 1: 
            self.active_robot = self.world.robot_test
            self.active_robot_calcs = self.robot_test_motion
        elif robot_id == 2:  #TODO: Add in the respective robots acording to their id
            active_robot = None
            active_robot_calcs = None
        elif robot_id == 3:
            pass
        elif robot_id == 4:
            pass
        
    
    def handle_gui(self):
        # Update GUI and process events
        if self.gui:
            self.gui.tick()
            events = self.gui.get_and_clear_events()
            
            # Handle GUI events
            for event_name, payload in events:
                
                if event_name == 'Start': self.Start.set_state(SS.ACTIVE)  
                elif event_name == 'estop': self.E_Stop.set_state(SS.ACTIVE)              
                elif event_name == 'estop_reset': self.Reset.set_state(SS.ACTIVE)
                
                elif event_name == 'stop_program':
                    self.running = False
                    break
                elif event_name == 'set_q' and payload is not None:
                    self.set_robot_to_move_GUIHelper(payload.get('robot_id'))
                    # Apply joint angles from GUI sliders to robot
                    q = payload.get('q')
                    self.active_robot.q = q
                elif event_name == 'jog_cart' and payload is not None: # Broken
                    self.set_robot_to_move_GUIHelper(payload.get('robot_id'))
                    
                    # Extract jog deltas from payload
                    dx = payload.get('dx', 0)
                    dy = payload.get('dy', 0)
                    dz = payload.get('dz', 0)
                    droll = payload.get('droll', 0)
                    dpitch = payload.get('dpitch', 0)
                    dyaw = payload.get('dyaw', 0)
                    
                    # Create a delta SE3 transform from jog values
                    delta_se3 = SE3(dx, dy, dz) * SE3.RPY([droll, dpitch, dyaw], order='xyz')
                    
                    # Get current end-effector pose via forward kinematics
                    se3_current = self.active_robot_calcs.forward_kinematics()
                    
                    # Compute new target pose by applying delta to current pose
                    se3_target = se3_current * delta_se3
                    
                    # Use inverse kinematics to get joint angles for new pose
                    q_new = self.active_robot_calcs.inverse_kinematics(se3_target)
                    
                    # Apply new joint angles to robot
                    self.active_robot.q = q_new
                     
                elif event_name == 'print_se3':
                    se3_current = self.active_robot_calcs.forward_kinematics()
                    print(f'{se3_current}')
                    self.counter=0
        
    def state_init(self):
        # Operation represents whatever overall test is being run
        self.OPERATION = State(SS.READY) 
        self.OPERATION_Counter = 0
        
        # gui buttons
        self.E_Stop = State(SS.DEACTIVE)
        self.Reset = State(SS.DEACTIVE)
        self.Start = State(SS.DEACTIVE)
                    
    def handle_states(self):
        """No operation are done here only state logic"""
        
        # Main operation controle
        if self.OPERATION.is_ready() and self.Start.is_active():
            self.OPERATION.set_state(SS.RUNNING)
            self.Start.set_state(SS.DEACTIVE) 
        elif self.OPERATION.is_achieved():
            self.OPERATION.set_state(SS.READY)
            self.OPERATION_Counter = 0
        
        # Button resets
        if self.Reset.is_active():
            self.OPERATION.set_state(SS.READY)
            self.Reset.set_state(SS.DEACTIVE)
            self.E_Stop.set_state(SS.DEACTIVE)
        elif self.E_Stop.is_active():
            self.OPERATION.set_state(SS.SUSPENDED)
            
        self.Start.set_state(SS.DEACTIVE)
        
            
    def handle_actions(self, inputs: dict):
        if self.OPERATION.is_running():
            self.world.robot_test.q = inputs.get("Test qtraj")[self.OPERATION_Counter]
            self.OPERATION_Counter += 1
            if self.OPERATION_Counter == 80:
                self.OPERATION.set_state(SS.ACHIEVED)
            
        
        
        
