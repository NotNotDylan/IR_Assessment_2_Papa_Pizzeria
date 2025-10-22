from external_e_stop import ExternalEStop
from imgui_GUI import GUIImGui
from manipulatable_object import ObjectNode
from movement_calculation import Robot1Movement, Robot2Movement, Robot3Movement, Robot4Movement, MovementCalculation
from states import State
from world import World
from states import State
from states import SystemState as SS
from states import PizzaStage  as PS


import time
import threading
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
from math import pi


# ... (other imports like numpy, logging, flask etc., as needed)

class Run:
    """Main controller for running the simulation loop and coordinating all components."""
    def __init__(self, world: World, gui: GUIImGui, estop: ExternalEStop, 
                 motions: list[Robot1Movement]):
        self.world = world
        self.gui = gui
        self.external_estop = estop
        self.motions = motions  # list of robot motion controller objects
        self.running = True
        self.running = True
        self.paused = False  # indicates if simulation is paused (e.g., after e-stop) # Initialize simulation time
        self.conveyerloop = 1
        self.last_stage = PS
        # Creating only one instance of the robot movment and calcuation objects
        
        self.robot1_motion = self.motions[0]
        self.robot2_motion = self.motions[1]
        self.robot3_motion = self.motions[2]
        self.robot4_motion = self.motions[3]
        
        self.dt = 0.05 # change in time between updates
        self.time_in_loop = 0
        
        self.joint_dict = {}
        
    
    def run_loop(self):
        """Run the main simulation loop, updating state, handling inputs, and moving robots."""
        # Simulation loop runs until `self.running` is False (could be set by GUI or other stop condition)

        # Initilises all the varable states used in the program
        self.state_init()
        
        # Generate toppings and pizza (The moveable kind)
        self.object_init()
        
        while self.running:
                        
            # Update states
            self.handle_states()
            
            # Update GUI and process events  
            self.handle_gui()
            
            # Calculations
            self.handle_calculations()
            
            # Effects are enacted
            self.handle_actions()
                
            # Step the environment to update visuals
            self.world.env.step(self.dt)
            
            # Small sleep to prevent excessive CPU usage
            time.sleep(0.01)
    
    def object_init(self):
        # Make initial pizza base
        self.pizza = ObjectNode(self.world.env, SE3(3.5, 3.6, 1.015), "Pizza's/Pizza_Base.stl", color=(0.90, 0.83, 0.70), name="Pizza")
        
        # Make the toppings
        self.cheese    = ObjectNode(self.world.env, SE3(6.5, 4, 1.004), "Pizza's/Pizza_Cheese.stl", color=(1.00, 0.78, 0.24), name="cheese")
        self.olives    = ObjectNode(self.world.env, SE3(6.2, 4, 1.01 ), "Pizza's/Olives.stl"      , color=(0.20, 0.20, 0.20), name="olives")
        self.ham       = ObjectNode(self.world.env, SE3(5.9, 4, 1    ), "Pizza's/Ham.stl"         , color=(1.00, 0.71, 0.76), name="ham")
        self.pepperoni = ObjectNode(self.world.env, SE3(6.8, 4, 1    ), "Pizza's/Pepperoni.stl"   , color=(0.71, 0.20, 0.14), name="pepperoni")
        self.pineapple = ObjectNode(self.world.env, SE3(7.1, 4, 1.005), "Pizza's/Pineapple.stl"   , color=(1.00, 0.90, 0.39), name="pineapple")
        self.box       = ObjectNode(self.world.env, SE3(7.5, 6.5, 1), "Pizza's/Pizza_Box2.stl"   , color=(1.0, 1.0, 1.0), name="box")
        self.motorbike = ObjectNode(self.world.env, SE3(3.0, 8.0, 0.0) * SE3.Rz(pi), "Environment/Honda Hornet STL.stl", color=(0.8, 0.8, 0.8), name="motorbike")
        
        # Adding to the world
        self.pizza.add_to_world()
        self.cheese.add_to_world() 
        self.olives.add_to_world() 
        self.ham.add_to_world() 
        self.pepperoni.add_to_world() 
        self.pineapple.add_to_world()
        self.box.add_to_world()
        self.motorbike.add_to_world()
    
    def handle_calculations(self):
        """
        Checks to see if there is a change from the last 
        state and runs the calculations for the next robot.
        """
        if self.last_stage != self.pizza_stage:
            match self.pizza_stage:
                case PS.ROBOT_1:
                    self.robot1_motion.get_ObjectNode(pizza=self.pizza)
                    self.joint_dict["Robot 1 Movment"] = self.robot1_motion.calculate()
                case PS.ROBOT_2:
                    self.robot2_motion.get_ObjectNode(pizza     = self.pizza,
                                                      cheese    = self.cheese,
                                                      olives    = self.olives,   
                                                      ham       = self.ham,      
                                                      pepperoni = self.pepperoni,
                                                      pineapple = self.pineapple,
                    )
                    self.joint_dict["Robot 2 Movment"] = self.robot2_motion.calculate()
                case PS.ROBOT_3:
                    self.robot3_motion.get_ObjectNode(pizza=self.pizza)
                    self.joint_dict["Robot 3 Movment"] = self.robot3_motion.calculate()
                case PS.ROBOT_4:
                    self.robot4_motion.get_ObjectNode(pizza=self.pizza)
                    self.joint_dict["Robot 4 Movment"] = self.robot4_motion.calculate()
                    
        self.last_stage = self.pizza_stage
        
        
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
        
        # Stages of operation
        self.pizza_stage = PS.FIRST_MOVE
        
        # Object State
        # self.robot1_state        = State(SS.DEACTIVE)
        # self.robot2_state        = State(SS.DEACTIVE)
        # self.robot3_state        = State(SS.DEACTIVE)
        # self.robot4_state        = State(SS.DEACTIVE)
        # self.motorcycle_state    = State(SS.DEACTIVE)
        # self.conveyerbelt_state  = State(SS.DEACTIVE)
        self.light_curtain_state = State(SS.DEACTIVE)
        
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
            # self.OPERATION_Counter = 0
        
        # internaly only itterates if operates is running
        self.handle_pizza_stage()
        
        # Button resets
        if self.Reset.is_active():
            self.OPERATION.set_state(SS.READY)
            self.Reset.set_state(SS.DEACTIVE)
            self.E_Stop.set_state(SS.DEACTIVE)
        elif self.E_Stop.is_active():
            self.OPERATION.set_state(SS.SUSPENDED)
            
        self.Start.set_state(SS.DEACTIVE)

    def handle_pizza_stage(self):
        """Handles which operations are occuring at the time"""
        match self.pizza_stage:
            case PS.FIRST_MOVE:
                self.pizza_stage_clock_helper(PS.ROBOT_1    , 22) # 55 when period was 5
            case PS.ROBOT_1:
                self.pizza_stage_clock_helper(PS.SECOND_MOVE, 81)
            case PS.SECOND_MOVE: 
                self.pizza_stage_clock_helper(PS.ROBOT_2    , 38) # 95
            case PS.ROBOT_2: 
                self.pizza_stage_clock_helper(PS.THIRD_MOVE , 240)
            case PS.THIRD_MOVE: 
                self.pizza_stage_clock_helper(PS.ROBOT_3    , 46) # 115
            case PS.ROBOT_3: 
                self.pizza_stage_clock_helper(PS.ROBOT_4    , 90)
            case PS.ROBOT_4: 
                self.pizza_stage_clock_helper(PS.MOTORCYCLE , 80)
            case PS.MOTORCYCLE: 
                self.pizza_stage_clock_helper(PS.COMPLETED  , 50)
            case PS.COMPLETED:
                pass
    
    def pizza_stage_clock_helper(self, next_stage: PS, counter: int):   
        if self.OPERATION.is_running():
            self.OPERATION_Counter += 1
            
        if self.OPERATION_Counter == counter:  # Operation will take this many steps
            self.OPERATION_Counter = 0
            self.pizza_stage = next_stage
    
    def conveyorBelt_step(self, period: int = 5): 
        if (self.OPERATION_Counter % period) == 0:
            if self.conveyerloop == 1: 
                self.world.plate1.T = self.world.plate1.T @ SE3( 0.1, 0, 0).A
                self.world.plate2.T = self.world.plate2.T @ SE3(-0.1, 0, 0).A
                self.conveyerloop = 2

            elif self.conveyerloop == 2:
                self.world.plate1.T = self.world.plate1.T @ SE3(-0.1, 0, 0).A
                self.world.plate2.T = self.world.plate2.T @ SE3( 0.1, 0, 0).A
                self.conveyerloop = 1   
            
    def handle_actions(self):
        
        # Disabled if estop or paused
        if self.OPERATION.is_running():
            
            # Assigning the precalculated joint angles to each of the robots
            if self.pizza_stage == PS.ROBOT_1:
                self.world.robot1.q = self.joint_dict.get("Robot 1 Movment")[self.OPERATION_Counter]
                # Materilise sauce
                if self.OPERATION_Counter == 49:                                                          # color=(0.698, 0.133, 0.133)
                    self.sauce = ObjectNode(self.world.env, SE3(0.0, 0.0, 0.0), "Pizza's/Pizza_Sauce.stl", color=(115/255, 9/255, 9/255), name="Sauce")
                    self.sauce.attach_to(self.pizza, keep_world_pose=False)
                    self.sauce.set_local_to_parent(SE3(0, 0, 0.0075))
                    self.sauce.add_to_world()
            elif self.pizza_stage == PS.ROBOT_2:
                fkine_result, q_traj = self.joint_dict.get("Robot 2 Movment")
                self.world.robot2.q = q_traj[self.OPERATION_Counter]
                
                if 30 <= self.OPERATION_Counter <= 50: # olives
                    self.olives.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 50:
                        self.olives.attach_to(self.pizza)
                        
                elif 70 <= self.OPERATION_Counter <= 90: # ham
                    self.ham.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 90:
                        self.ham.attach_to(self.pizza)
                        
                elif 110 <= self.OPERATION_Counter <= 130: # pepperoni
                    self.pepperoni.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 130:
                        self.pepperoni.attach_to(self.pizza)
                        
                elif 150 <= self.OPERATION_Counter <= 170: # pineapple
                    self.pineapple.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 170:
                        self.pineapple.attach_to(self.pizza)
                        
                elif 190 <= self.OPERATION_Counter <= 210: # cheese
                    self.cheese.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 210:
                        self.cheese.attach_to(self.pizza)
                        
            elif self.pizza_stage == PS.ROBOT_3:
                fkine_result, q_traj = self.joint_dict.get("Robot 3 Movment")
                self.world.robot3.q = q_traj[self.OPERATION_Counter]
                
                if 15 <= self.OPERATION_Counter <= 75:
                    self.pizza.set_pose(fkine_result[self.OPERATION_Counter])
                    if self.OPERATION_Counter == 75:
                        self.pizza.attach_to(self.box)
                
            elif self.pizza_stage == PS.ROBOT_4:
                self.world.robot4.q = self.joint_dict.get("Robot 4 Movment")[self.OPERATION_Counter]
            
            # Anamates the conveyerbelt & moves pizza
            elif self.pizza_stage == PS.FIRST_MOVE or PS.SECOND_MOVE or PS.THIRD_MOVE:
                self.conveyorBelt_step(period=2)
                self.pizza.move_by(SE3.Tx(0.1/2))
                
            elif self.pizza_stage == PS.MOTORCYCLE:
                pass
            
            elif self.pizza_stage == PS.COMPLETED:
                pass
            
            
        
        
        
