from manipulatable_object import ObjectNode

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
    
    def update(self):
        """Placeholder update method to be overridden by subclasses. Computes next motion based on system state."""
        pass  # To be implemented in each subclass


class Robot1Movement(MovementCalculation):
    """Controls Robot 1 (Sauce application robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.sauce_applied = False  # flag to indicate if sauce task done for current pizza
    
    def calculate(self):
        """
        Zero position
        Circle above pizza (Three points untill RMRC)
        Zero position
        """
        # Joint anges at each step
        q_step1 = self.robot.q  # initial configuration
        q_step2 = self.inverse_kinematics(SE3(4.6 + 0.00, 3.6 + 0.12, 1.015) @ SE3.Ry(np.pi), q_step1)  # Three points of a triangle (circle)
        q_step3 = self.inverse_kinematics(SE3(4.6 - 0.10, 3.6 - 0.06, 1.015) @ SE3.Ry(np.pi), q_step2)
        q_step4 = self.inverse_kinematics(SE3(4.6 + 0.10, 3.6 - 0.06, 1.015) @ SE3.Ry(np.pi), q_step3)
        q_step5 = q_step2 # This step completes the circle
        q_step6 = q_step1
        
        # Calculate trajectories
        q_traj1 = rtb.jtraj(q_step1, q_step2, 70).q
        q_traj2 = rtb.jtraj(q_step2, q_step3, 20).q
        q_traj3 = rtb.jtraj(q_step3, q_step4, 20).q
        q_traj4 = rtb.jtraj(q_step4, q_step5, 20).q
        q_traj5 = rtb.jtraj(q_step5, q_step6, 70).q
        
        # Join togther trajectories
        q_traj_final = np.concatenate([q_traj1, q_traj2, q_traj3, q_traj4, q_traj5], axis=0)
        
        return q_traj_final # 70+20+20+20+70 = 200 steps

class Robot2Movement(MovementCalculation):
    """Controls Robot 2 (Topping placement robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.toppings_placed = False  # or count of toppings placed
    
    def calculate(self):
        """
        Zero position
        pick place topping
        pick place topping
        pick place topping
        Zero position
        """
        pass

class Robot3Movement(MovementCalculation):
    """Controls Robot 3 (Oven handling robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.pizza_in_oven = False
    
    def calculate(self):
        """
        pick up pizza
        place in oven
        wait outside
        pick up pizza
        place in box
        """
        pass

class Robot4Movement(MovementCalculation):
    """Controls Robot 4 (Packaging and loading robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
        self.box_ready = False
    
    def calculate(self):
        """
        pick up pizza box
        place on motorcycle
        """
        pass