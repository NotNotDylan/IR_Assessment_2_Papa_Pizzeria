from external_e_stop import ExternalEStop
from imgui_GUI import GUIImGui
from logger import Logger
import Simple_Gripper.manipulatable_object as manipulatable_object
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
from ir_support.robots.DHRobot3D import DHRobot3D


# ... (other imports like numpy, logging, flask etc., as needed)

class Run:
    """Main controller for running the simulation loop and coordinating all components."""
    def __init__(self, world: World, gui: GUIImGui, estop: ExternalEStop, 
                 logger: Logger, motions: list):
        self.world = world
        self.gui = gui
        self.external_estop = estop
        self.logger = logger
        self.motions = motions  # list of robot motion controller objects
        self.running = True
        self.running = True
        self.paused = False  # indicates if simulation is paused (e.g., after e-stop) # Initialize simulation time
        self.loop = 1
        self.loop2 = 1
        # Creating only one instance of the robot movment and calcuation objects
        # self.robot_test_motion = MovementCalculation(self.world.robot_test)
        self.robot1_motion = self.motions[0]
        self.robot2_motion = self.motions[1]
        self.robot3_motion = self.motions[2]
        self.robot4_motion = self.motions[3]
        
        self.dt = 0.05 # change in time between updates
        self.time_in_loop = 0
        
        self.actions_inputs = {}
        
    
    def run_loop(self):
        """Run the main simulation loop, updating state, handling inputs, and moving robots."""
        # Simulation loop runs until `self.running` is False (could be set by GUI or other stop condition)
        qtraj = self.test_inverse_kinamatics()
        
        self.actions_inputs["Test qtraj"] = qtraj
        
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
            self.handle_actions(self.actions_inputs)
                
            # Step the environment to update visuals
            self.world.env.step(self.dt)
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(0.01)
            
        
        # End of loop - clean up if necessary (e.g., close env if not holding)
        # (The world.env.hold() call can be done after this function returns.)
    
    def test_inverse_kinamatics(self):
        curent_q = self.world.robot1.q
        
        goal = SE3(0.5788,1.27,0.8437)
        goal_q = self.robot1_motion.inverse_kinematics(goal)
        
        qtraj = rtb.jtraj(curent_q, goal_q, 80).q
        return qtraj
        
    def set_robot_to_move_GUIHelper(self, robot_id):
        if robot_id == 1: 
            self.active_robot = self.world.robot1
            self.active_robot_calcs = self.robot1_motion
        elif robot_id == 2:
            self.active_robot = self.world.robot2
            self.active_robot_calcs = self.robot2_motion
        elif robot_id == 3:
            self.active_robot = self.world.robot3
            self.active_robot_calcs = self.robot3_motion
        elif robot_id == 4:
            self.active_robot = self.world.robot4
            self.active_robot_calcs = self.robot4_motion
        
    
    def handle_gui(self):
        
        is_paused = (self.OPERATION.is_suspended() or not self.OPERATION.is_running())
        self.gui.set_system_paused(is_paused)
        
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
        
        # Hardware e-stop poll (fail-safe if link stale)
        if self.external_estop and self.external_estop.check_stop():
            self.E_Stop.set_state(SS.ACTIVE)
            self.gui._estop_latched = True
        
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
            self.world.robot1.q = inputs.get("Test qtraj")[self.OPERATION_Counter]
            self.OPERATION_Counter += 1
            if self.OPERATION_Counter == 80:
                self.OPERATION.set_state(SS.ACHIEVED)
            
        
        
        
