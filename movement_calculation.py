from states import States
from manipulatable_object import Topping

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal, DHRobot
from scipy import linalg
from numpy import pi

class MovementCalculation:
    """Base class for robot movement calculations: kinematics, path planning, and safety checks."""
    def __init__(self, robot_model: rtb.DHRobot):
        self.robot = robot_model  # The RTB robot model (or custom model) this controller operates
        self.target_joint_positions = None  # For planned joint trajectory or target position
        self.current_task = None   # Description of current task (if any)
        self.collision_avoided = False  # Flag if we had to adjust for collision in last step
    
    def forward_kinematics(self, q=None):
        """Return the end-effector pose (SE3) for given joint angles q. If q not provided, uses robot's current q."""
        if q is None:
            q = self.robot.q  # current joint angles
        # Use RTB forward kinematics:
        return self.robot.fkine(q)
    
    def inverse_kinematics(self, target_pose: SE3, q_seed=None):
        """Solve inverse kinematics for the robot to reach the target_pose. Returns joint angles solution."""
        # TODO: Use RTB's IK solver or implement one.
        # e.g., sol = self.robot.ikine_LMS(target_pose)  (if using RTB's ikine methods)
        # return sol.q
        
        # Seed: current joints if none provided
        if q_seed is None:
            q_seed = np.asarray(self.robot.q, dtype=float)
        else:
            q_seed = np.asarray(q_seed, dtype=float)

        sol = self.robot.ikine_LM(
                target_pose,
                q0=q_seed,
                ilimit=30, # Max number of itterations (Reduce for faster but may not get within tolarance)
                tol=1e-6, # Once within tolarance it will stop itterating inverse kinimatics
                mask=np.array([1, 1, 1, 1, 1, 1]),
                joint_limits=True # reject if joints violate self.robot.qlim
            )
        
        return np.asarray(sol, dtype=float)
    
    def avoid_collisions(self, q_candidate):
        """Collision check for a candidate joint configuration or path. 
        Returns True if a collision is detected and avoidance action is needed."""
        # TODO: Implement collision checking logic.
        # You could use bounding boxes or simple distance thresholds between robot links and known obstacles (including other robots or the intruding object).
        # If collision predicted, you might modify q_candidate or set a flag to delay motion.
        # For now, return False (no collision) by default.
        
        return False
    
    def emergency_stop(self):
        """Immediate stop: can be called to halt the robot. Could set target to current position (no motion)."""
        # Simple implementation: just set target to current to freeze movement.
        self.target_joint_positions = self.robot.q
    
    def update(self, state: States):
        """Placeholder update method to be overridden by subclasses. Computes next motion based on system state."""
        pass  # To be implemented in each subclass

    def collision_detected(self, point, array=None):
        for i in array:
            if array[i][0] <= point[0] <= array[i][1]:
                if array[i][2] <= point[1] <= array[i][3]:
                    if array[i][4] <= point[2] <= array[i][5]:
                        return True
                    
    def RMRC(self, next_pos):
        self._tool = None
        
        q = self.robot.q
        T1 = self.robot.fkine(q)
        x1 = T1.t

        T2 = next_pos
        x2 = T2.t

        roll, pitch, yaw = next_pos.rpy(unit="rad", order="zyx")

        delta_t = 0.05
        steps = 100

        x = np.zeros([3,steps])
        s = trapezoidal(0,1,steps).q

        for tryes in range(steps):
            x[:,i] = x1*(1-s[i]) + s[i]*x2

        for i in range(20):
            q_matrix = np.zeros([steps, 6])

            q_matrix[0,:] = self.robot.ikine_LM(T1).q

            # Finds path of robot movment (as joint positions)
            for i in range(steps-1):                     # Calculate velocity at discrete time step
                J = self.robot.jacob0(q_matrix[i, :])  # 6x6 full Jacobian
                xdot_linear = (x[:, i+1] - x[:, i]) / delta_t  # 3x1
                xdot_angular = np.array(roll/(steps*delta_t), pitch/(steps*delta_t), yaw/(steps*delta_t))                # keep orientation fixed
                xdot_full = np.hstack((xdot_linear, xdot_angular))
                q_dot = np.linalg.pinv(J) @ xdot_full
                q_matrix[i+1,:] = q_matrix[i,:] + delta_t * q_dot

            points = []  # collect all FK positions

            # Find's each joint position in the arm relative to world
            for q in q_matrix:
                S = self.robot.fkine_path(q)   
                for T in S:     
                    point = T.t
                    points.append(point)

            # Checks all joint positioins to see if they are within a colision box/bound
            for i in range(len(points)):
                if self.collision_detected(point=points[i]) == False:
                    break

            if tryes == 19:
                print("RMRC failed to find trajectory in 20 tries")
        
        for q in q_matrix:
            self.q = q
            


                


            






class Robot1Movement(MovementCalculation):
    """Controls Robot 1 (Sauce application robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.sauce_applied = False  # flag to indicate if sauce task done for current pizza
    
    def update(self, state: States):
        """Update Robot1 actions: if a pizza is ready for sauce, perform the sauce dispensing routine."""
        # If Robot1 is free and a pizza is ready for sauce:
        for pizza in state.pizzas:
            if pizza.stage == "ready_for_sauce" and not self.sauce_applied:
                # Move to sauce dispense position above the pizza
                target_pose = pizza.mesh.pose * SE3(0, 0, 0.3)  # e.g., 30 cm above pizza center
                # Compute IK for that position
                q_target = self.inverse_kinematics(target_pose)
                if q_target is not None:
                    # Check for singularities or unreachable config
                    if self.detect_singularity(self.compute_jacobian(q_target)):
                        # If near singular, maybe adjust or use damped IK
                        q_target = self.inverse_kinematics(target_pose)  # (could use different method or damping)
                    # Collision avoidance check (e.g., ensure not colliding with pizza or environment)
                    if self.avoid_collisions(q_target):
                        # If collision would occur, adjust path (for simplicity, skip or raise arm)
                        adjusted_pose = pizza.mesh.pose * SE3(0, 0, 0.5)
                        q_target = self.inverse_kinematics(adjusted_pose)
                    # Command robot to move (here, just set target joints)
                    self.target_joint_positions = q_target
                    self.robot.q = q_target  # directly setting for instant move in simulation (or interpolate in steps)
                    # Simulate sauce dispensing action (could be a delay or a small spiral motion)
                    # e.g., you might do a small circle with end-effector over pizza:
                    # (skipping actual path, but in a real case you'd generate a trajectory or use visual servo to spread sauce)
                    self.sauce_applied = True
                    pizza.stage = "sauce_done"
                    state.robot_busy[1] = False  # free robot1 after done
                    # Reactivate conveyor 1 to send pizza to next station
                    if len(self.robot.q) > 0:  # dummy check to avoid error
                        state.world.conveyors[0].active = True
                else:
                    # IK failed (target not reachable?), consider this an error or adjust pose.
                    self.robot.q = self.robot.q  # no change (could log an error)
        # If no pizza ready or already handled, Robot1 remains idle.
        return

class Robot2Movement(MovementCalculation):
    """Controls Robot 2 (Topping placement robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.toppings_placed = False  # or count of toppings placed
    
    def update(self, state: States):
        """Update Robot2 actions: place toppings on pizza when ready."""
        for pizza in state.pizzas:
            if pizza.stage == "ready_for_toppings" and not self.toppings_placed:
                # Example procedure: pick up a topping from a bin, place on pizza, repeat for all toppings.
                # You might have predefined bin locations for toppings.
                toppings_to_add = ["Topping1", "Topping2"]  # example list of topping types
                for topping_type in toppings_to_add:
                    # Get or create a topping object from the pizza or environment
                    topping = Topping(type=topping_type)
                    # Spawn topping at bin location (you could have pre-placed them and just pick one)
                    topping.mesh.pose = SE3(...)  # TODO: position at bin pickup location
                    state.world.add_object(topping)
                    # Move robot to pick topping
                    pickup_pose = topping.mesh.pose * SE3(0, 0, 0.1)  # above the topping
                    q_pick = self.inverse_kinematics(pickup_pose)
                    if q_pick is not None:
                        self.robot.q = q_pick  # move to pickup
                        # (Simulate gripper closing on topping - could just attach topping to robot in simulation)
                        # Move to place topping on pizza
                        place_pose = pizza.mesh.pose * SE3(0, 0, 0.05)  # just above pizza surface
                        q_place = self.inverse_kinematics(place_pose)
                        if q_place is not None:
                            self.robot.q = q_place
                            # Release topping (place it) by leaving it in environment on pizza
                            topping.mesh.pose = pizza.mesh.pose * SE3(0, 0, 0.01)  # position topping on pizza
                            topping.attached_to_pizza = pizza
                        # (In a real sim, remove topping from robot gripper; here we just reposition it)
                    # End loop for one topping
                # After placing all toppings:
                self.toppings_placed = True
                pizza.stage = "toppings_done"
                state.robot_busy[2] = False
                # Restart conveyor 2 to move pizza to next station (oven)
                state.world.conveyors[1].active = True
        return

class Robot3Movement(MovementCalculation):
    """Controls Robot 3 (Oven handling robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.pizza_in_oven = False
    
    def update(self, state: States):
        """Update Robot3 actions: load pizza into oven, wait, and unload it."""
        for pizza in state.pizzas:
            if pizza.stage == "ready_for_oven" and not self.pizza_in_oven:
                # Move to pick pizza from conveyor 3
                pick_pose = pizza.mesh.pose * SE3(0, 0, 0.1)  # just above pizza
                q_pick = self.inverse_kinematics(pick_pose)
                if q_pick is not None:
                    self.robot.q = q_pick
                    # Simulate picking up the pizza (attach to robot's "spatula")
                    # Remove pizza from conveyor visually: (or mark it as being carried)
                    state.world.remove_object(pizza)
                    # Move to oven entry position
                    oven_entry_pose = SE3(...)  # predefined pose just in front of oven
                    q_oven = self.inverse_kinematics(oven_entry_pose)
                    if q_oven is not None:
                        self.robot.q = q_oven
                        # Place pizza into oven (in sim, you might just wait a moment)
                        self.pizza_in_oven = True
                        pizza.stage = "in_oven"
                        # (You could add a delay or loop to simulate baking time)
                        # After baking time:
                        pizza.stage = "baked"
                        # Pick pizza out of oven (same pose as entry for simplicity)
                        self.robot.q = q_oven
                        # Re-add pizza object to scene as if taken out
                        state.world.add_object(pizza)
                        pizza.mesh.pose = oven_entry_pose  # now pizza is at oven exit
                        self.pizza_in_oven = False
                        pizza.stage = "out_of_oven"
                        state.robot_busy[3] = False
                        # Reactivate conveyor 3 to move pizza to final station (if there's a small output conveyor)
                        state.world.conveyors[2].active = True
        return

class Robot4Movement(MovementCalculation):
    """Controls Robot 4 (Packaging and loading robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.box_ready = False
    
    def update(self, state: States):
        """Update Robot4 actions: pack pizza into box and load onto motorbike."""
        for pizza in state.pizzas:
            if pizza.stage == "ready_for_pack":
                # If not already in a box, place pizza in box
                if not self.box_ready:
                    # Create a box object and place pizza in it (for simplicity, we just assume box is at robot station)
                    pizza.in_box = True  # mark pizza as boxed (you could swap the pizza mesh to a box mesh)
                    self.box_ready = True
                # Now lift the boxed pizza and place it on the motorbike
                target_pose = state.world.motorbike.pose * SE3(0, 0, 0.5)  # e.g., above bike's storage area
                # Visual servoing could be used here to adjust final placement:
                # For example, use a camera to find the exact bike rack position. Simulate by fine-tuning the target pose.
                # TODO: Implement visual servoing loop: repeatedly get current end-effector pose and compute small adjustments until aligned.
                q_target = self.inverse_kinematics(target_pose)
                if q_target is not None:
                    self.robot.q = q_target
                    # Release the box (pizza) on the bike
                    pizza.mesh.pose = state.world.motorbike.pose * SE3(0, 0, 0.1)  # put pizza box on bike seat
                    pizza.delivered = True
                    pizza.stage = "loaded_on_bike"
                    state.robot_busy[4] = False
                    # Simulate motorbike driving off with the pizza
                    # You might animate the bike or just remove the pizza:
                    state.world.remove_object(pizza)
                # Reset box_ready for next pizza
                self.box_ready = False
        return
