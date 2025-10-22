from manipulatable_object import ObjectNode

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal, DHRobot, jtraj
from scipy import linalg
from numpy import pi

class MovementCalculation:
    """Base class for robot movement calculations: kinematics, path planning, and safety checks."""
    def __init__(self, robot_model: rtb.DHRobot):
        self.robot = robot_model  # The RTB robot model (or custom model) this controller operates
        self.target_joint_positions = None  # For planned joint trajectory or target position
        self.current_task = None   # Description of current task (if any)
        self.collision_avoided = False  # Flag if we had to adjust for collision in last step
        
    def get_ObjectNode(self, object: ObjectNode):
        """Only stores the most recently called one"""
        self.object = object
    
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

    # def collision_detected(self, point):
    #     array = self.bounds_array
        
    #     for i in array:
    #         if array[i][0] <= point[0] <= array[i][1]:
    #             if array[i][2] <= point[1] <= array[i][3]:
    #                 if array[i][4] <= point[2] <= array[i][5]:
    #                     return True

    def trajectoryCheck(self, trajectory, r):
        self.bounds_array = np.array([
            [3.0, 4.0, 3.0, 4.2, 0.0, 1.48],
            [4.0, 9.0, 3.4, 3.8, 0.0, 0.98],
            [7.0, 8.0, 5.7435, 7.2435, 0.0, 0.98],
            [10.483, 13.483, 6.3631, 9.3631, 0.0, 0.98],
            [3.10, 7.32, 4.725, 4.775, 0.0, 2.98],
            [5.6486, 6.0486, 6.2944, 6.6944, 0.0, 0.48],
            [9.52, 9.92, 5.4, 5.8, 0.0, 0.48],
            [4.4, 4.8, 3.85, 4.25, 0.0, 0.98],
            [6.32, 6.72, 4.2, 4.6, 0.0, 0.98],
            [6.0, 7.0, 3.875, 4.125, 0.0, 0.98]
        ])

        for q in trajectory.q:
            allLinks = r.fkine_all(q)  # get all link transforms
            for T in allLinks:
                x, y, z = T.t
                for zone in self.bounds_array:
                    xmin, xmax, ymin, ymax, zmin, zmax = zone
                    if xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax:
                        return True
        return False
    
    def trajectory(self,start_q,target_pose,steps):
        q_target = self.ikine_LM(target_pose, q0=start_q, joint_limits=True).q
        traj = jtraj(start_q, q_target, steps)

        # Replan if trajectory crosses forbidden region
        attempt = 0
        while self.trajectoryCheck(traj, self): #and attempt < 10:
            attempt += 1
            print(f"Trajectory invalid — replanning (attempt {attempt})...")
            q_guess = start_q + np.random.uniform(-0.1, 0.1, size=len(self.q))
            q_target = self.ikine_LM(target_pose, q0=q_guess, joint_limits=True).q
            traj = jtraj(start_q, q_target, steps)
            print("Trying")

        if attempt >= 10:
            print("No valid trajectory found after 10 attempts.")
            return

        # Animate the valid trajectory
        print("Animating valid trajectory...")
        for q in traj.q:
            self.q = q
    
    def collision_detected(self, point):
        bounds = self.bounds_array  # don't shadow "array"
        x, y, z = float(point[0]), float(point[1]), float(point[2])

        for box in bounds:  # box = [xmin, xmax, ymin, ymax, zmin, zmax]
            if box[0] <= x <= box[1] and box[2] <= y <= box[3] and box[4] <= z <= box[5]:
                return True
        return False
                    
    def RMRC(self, inital_pos: SE3, next_pos: SE3, steps: int):
        # self._tool = None    
        self.robot.tool = self.robot.tool if isinstance(self.robot.tool, SE3) else SE3()  # ensure SE3     
        
        T1 = inital_pos
        x1 = T1.t

        T2 = next_pos
        x2 = T2.t

        roll, pitch, yaw = next_pos.rpy(unit="rad", order="zyx")

        delta_t = 0.05

        x = np.zeros([3,steps])
        s = trapezoidal(0,1,steps).q

        for i in range(steps):
            x[:,i] = x1*(1-s[i]) + s[i]*x2

        for tryes in range(6):
            q_matrix = np.zeros([steps, 6])

            q_matrix[0,:] = self.robot.ikine_LM(T1).q

            # Finds path of robot movment (as joint positions)
            for i in range(steps-1):                     # Calculate velocity at discrete time step
                J = self.robot.jacob0(q_matrix[i, :])  # 6x6 full Jacobian
                xdot_linear = (x[:, i+1] - x[:, i]) / delta_t  # 3x1
                xdot_angular = np.array([0,0,0])                # keep orientation fixed
                xdot_full = np.hstack((xdot_linear, xdot_angular))
                q_dot = np.linalg.pinv(J) @ xdot_full
                q_matrix[i+1,:] = q_matrix[i,:] + delta_t * q_dot

            points = []  # collect all FK positions

            # Find's each joint position in the arm relative to world
            for q in q_matrix:
                # S = self.robot.fkine_path(q)
                S = self.robot.fkine_all(q)   
                for T in S:     
                    point = T.t
                    points.append(point)

            # Checks all joint positioins to see if they are within a colision box/bound
            for i in range(len(points)):
                if self.collision_detected(point=points[i]) == False:
                    break

            if tryes == 5:
                print("RMRC failed to find trajectory in 6 tries")
        
        return q_matrix
            

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
        
        pizza_cord = self.object.xyz_of_node()
        
        # # Old method
        # # Joint anges at each step
        # q_step1 = self.robot.q  # initial configuration
        # q_step2 = self.inverse_kinematics(SE3(pizza_cord[0] + 0.00, pizza_cord[1] + 0.12, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step1)  # Three points of a triangle (circle)
        # q_step3 = self.inverse_kinematics(SE3(pizza_cord[0] - 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step2)
        # q_step4 = self.inverse_kinematics(SE3(pizza_cord[0] + 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step3)
        # q_step5 = q_step2 # This step completes the circle
        # q_step6 = q_step1
        
        # # Calculate trajectories
        # q_traj1 = self.trajectory(self.robot.q, SE3(pizza_cord[0] + 0.00, pizza_cord[1] + 0.12, pizza_cord[2] + 0.03), 70).q
        # q_traj2 = rtb.jtraj(q_step2, q_step3, 20).q
        # q_traj3 = rtb.jtraj(q_step3, q_step4, 20).q
        # q_traj4 = rtb.jtraj(q_step4, q_step5, 20).q
        # q_traj5 = rtb.jtraj(q_step5, q_step6, 70).q
        
        # SE3 end effector at each location for each step       
        # se3_step1 = self.forward_kinematics()  # initial SE3 configuration
        # se3_step2 = SE3(pizza_cord[0] + 0.00, pizza_cord[1] + 0.12, pizza_cord[2] + 0.03) # @ SE3.Ry(np.pi)
        # se3_step3 = SE3(pizza_cord[0] - 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) # @ SE3.Ry(np.pi)
        # se3_step4 = SE3(pizza_cord[0] + 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) # @ SE3.Ry(np.pi)
        # se3_step5 = se3_step2 # This step completes the circle/triangle
        # se3_step6 = se3_step1
        
        # # Calculate trajectories
        # q_traj1 = self.RMRC(se3_step1, se3_step2, 70)
        # q_traj2 = self.RMRC(se3_step2, se3_step3, 20)
        # q_traj3 = self.RMRC(se3_step3, se3_step4, 20)
        # q_traj4 = self.RMRC(se3_step4, se3_step5, 20)
        # q_traj5 = self.RMRC(se3_step5, se3_step6, 70)
        
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