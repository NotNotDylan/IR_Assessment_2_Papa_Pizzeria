from manipulatable_object import ObjectNode

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