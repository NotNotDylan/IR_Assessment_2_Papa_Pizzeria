from states import States
from manipulatable_object import Topping

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb

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
            q_seed = self.robot.q
        # else:
        #     q_seed = q_seed

        sol = self.robot.ikine_LM(
                target_pose,
                q0=q_seed,
                ilimit=30, # Max number of itterations (Reduce for faster but may not get within tolarance)
                tol=1e-6, # Once within tolarance it will stop itterating inverse kinimatics
                mask=np.array([1, 1, 1, 1, 1, 1]),
                joint_limits=True # reject if joints violate self.robot.qlim
            )
        
        return sol.q
    
    def compute_jacobian(self, q=None):
        """Compute the robot Jacobian at joint configuration q (or current q)."""
        if q is None:
            q = self.robot.q
        # If using RTB, many models have a .jacob0 or .jacobe method for Jacobian
        try:
            J = self.robot.jacob0(q)
        except Exception:
            J = self.robot.jacob0(q, end=None)  # depending on RTB version, might need specifying end link
        return J
    
    def detect_singularity(self, J=None):
        """Check manipulability and return True if near singularity."""
        if J is None:
            J = self.compute_jacobian()
        # Compute manipulability measure (sigma_min or sqrt(det(J*J^T)) etc.)
        try:
            # One measure: product of singular values (which is sqrt(det(JJ^T)))
            U, S, V = np.linalg.svd(J)
            manipulability = np.prod(S)
        except Exception:
            # If J is not square etc., use det(JJT) if possible:
            JJt = J @ J.T
            manipulability = np.sqrt(np.linalg.det(JJt)) if JJt.shape[0] == JJt.shape[1] else 0
        # Define a threshold (epsilon) for near singularity
        epsilon = 1e-3  # TODO: adjust threshold as appropriate
        return manipulability < epsilon
    
    def damped_pseudoinverse(self, J, lambda_coeff=0.1):
        """Compute the damped least squares pseudoinverse of Jacobian J."""
        # DLS formula: J_damped_inv = J^T * (J*J^T + lambda^2 I)^-1
        JJt = J @ J.T
        n = JJt.shape[0]
        damp_matrix = JJt + (lambda_coeff**2) * np.eye(n)
        inv_damp = np.linalg.inv(damp_matrix)
        J_damped_inv = J.T @ inv_damp
        return J_damped_inv
    
    def compute_velocity_command(self, current_pose: SE3, desired_pose: SE3, dt: float):
        """Compute end-effector velocity needed to move from current_pose to desired_pose in time dt."""
        # Compute position error
        pos_error = desired_pose.t - current_pose.t  # 3x1 vector difference in translation
        # Compute orientation error (for simplicity, using Euler angles or rotation matrix difference)
        Rd = desired_pose.R
        Ra = current_pose.R
        R_err = Rd @ Ra.T  # rotation from current to desired
        # Convert rotation error to axis-angle or equivalent (small-angle approximation for velocity)
        # For a small rotation, we can extract axis-angle from R_err (or use log map).
        # Here, we just take the skew-symmetric part as angular velocity * dt
        delta_theta = np.array([0, 0, 0])
        # TODO: extract small rotation (e.g., using spatialmath's Log or angle between orientations)
        # Simplification: assume orientation errors are small, so use difference of R matrices
        # (Better: use `smbase.tr2rpy(R_err)` or similar to get roll-pitch-yaw error.)
        # 
        # Now form the end-effector velocity vector (6x1) [vx, vy, vz, wx, wy, wz]^T
        linear_vel = pos_error * (1.0/dt)
        angular_vel = delta_theta * (1.0/dt)
        xdot = np.hstack((linear_vel, angular_vel))
        return xdot
    
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
