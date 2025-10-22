from manipulatable_object import ObjectNode

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal, DHRobot, jtraj
from scipy import linalg
from numpy import pi
import swift

class MovementCalculation:
    """Base class for robot movement calculations: kinematics, path planning, and safety checks."""
    def __init__(self, robot_model: rtb.DHRobot):
        self.robot = robot_model  # The RTB robot model (or custom model) this controller operates
        self.target_joint_positions = None  # For planned joint trajectory or target position
        self.current_task = None   # Description of current task (if any)
        self.collision_avoided = False  # Flag if we had to adjust for collision in last step
        
        self.bounds_array = np.array([
            [3.0, 4.0, 3.0, 4.2, 0.0, 1.5],
            [4.0, 9.0, 3.4, 3.8, 0.0, 1.0],
            [7.0, 8.0, 5.7435, 7.2435, -0.024095, 0.975905],
            [10.483, 13.483, 6.3631, 9.3631, 0.0, 1.0],
            [3.10, 7.32, 4.725, 4.775, 0.0, 3.0],
            [5.6486, 6.0486, 6.2944, 6.6944, 0.0, 0.5],
            [9.52, 9.92, 5.4, 5.8, 0.0, 0.5],
            [4.4, 4.8, 3.85, 4.25, 0.0, 1.0],
            [6.32, 6.72, 4.2, 4.6, 0.0, 1.0],
            [6.0, 7.0, 3.875, 4.125, 0.0, 1.0]
        ])
        
    def get_ObjectNode(self, 
                       pizza:     ObjectNode = None,
                       cheese:    ObjectNode = None,
                       olives:    ObjectNode = None,
                       ham:       ObjectNode = None,
                       pepperoni: ObjectNode = None,
                       pineapple: ObjectNode = None
        ):
        """Grabs the Object node so it can be manipulated with here"""
        self.pizza     = pizza
        self.cheese    = cheese   
        self.olives    = olives   
        self.ham       = ham      
        self.pepperoni = pepperoni
        self.pineapple = pineapple
    
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
    
    def calculate(self):
        """
        Zero position
        Circle above pizza
        Zero position
        """
        
        pizza_cord = self.pizza.xyz_of_node()
        
        # # Old method
        # # Joint anges at each step
        q_step1 = self.robot.q  # initial configuration
        q_step2 = self.inverse_kinematics(SE3(pizza_cord[0] + 0.00, pizza_cord[1] + 0.12, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step1)  # Three points of a triangle (circle)
        q_step3 = self.inverse_kinematics(SE3(pizza_cord[0] - 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step2)
        q_step4 = self.inverse_kinematics(SE3(pizza_cord[0] + 0.10, pizza_cord[1] - 0.06, pizza_cord[2] + 0.03) @ SE3.Ry(np.pi), q_step3)
        q_step5 = q_step2 # This step completes the circle
        q_step6 = q_step1
        
        # Calculate trajectories
        q_traj1 = rtb.jtraj(q_step1, q_step2, 30).q
        q_traj2 = rtb.jtraj(q_step2, q_step3, 7).q
        q_traj3 = rtb.jtraj(q_step3, q_step4, 7).q
        q_traj4 = rtb.jtraj(q_step4, q_step5, 7).q
        q_traj5 = rtb.jtraj(q_step5, q_step6, 30).q
        
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
        
        return q_traj_final # (30*2)+(7*3) = 81 steps

class Robot2Movement(MovementCalculation):
    """Controls Robot 2 (Topping placement robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
    
    def calculate(self):
        """
        Zero position
        pick place topping
        pick place topping
        pick place topping
        Zero position
        """
        # Getting initial cordanates
        pizza_cord     = self.pizza.xyz_of_node()
        cheese_cord    = self.cheese.xyz_of_node()
        olives_cord    = self.olives.xyz_of_node()
        ham_cord       = self.ham.xyz_of_node()
        pepperoni_cord = self.pepperoni.xyz_of_node()
        pineapple_cord = self.pineapple.xyz_of_node()
        
        
        # Joint anges at each step
        q_step1    = self.robot.q  # initial configuration                                                                    
        q_step2    = self.inverse_kinematics(SE3(olives_cord[0]    + 0.00, olives_cord[1]    + 0.00, olives_cord[2]    + 0.1 + 0.025 ) @ SE3.Ry(np.pi), q_step1  )
        q_step2_1  = self.inverse_kinematics(SE3(6.52, 3.8, 1.4) @ SE3.Ry(np.pi), q_step2)                                
        q_step3    = self.inverse_kinematics(SE3(pizza_cord[0]     + 0.00, pizza_cord[1]     + 0.00, pizza_cord[2]     + 0.1 + 0.015) @ SE3.Ry(np.pi), q_step2  ) 
        q_step3_1  = q_step2_1                                                                                                 
        q_step4    = self.inverse_kinematics(SE3(ham_cord[0]       + 0.00, ham_cord[1]       + 0.00, ham_cord[2]       + 0.1 + 0.015 ) @ SE3.Ry(np.pi), q_step2_1) 
        q_step4_1  = q_step2_1
        q_step5    = self.inverse_kinematics(SE3(pizza_cord[0]     + 0.00, pizza_cord[1]     + 0.00, pizza_cord[2]     + 0.1 + 0.015 ) @ SE3.Ry(np.pi), q_step2_1)
        q_step5_1  = q_step2_1
        q_step6    = self.inverse_kinematics(SE3(pepperoni_cord[0] + 0.00, pepperoni_cord[1] + 0.00, pepperoni_cord[2] + 0.1 + 0.02 ) @ SE3.Ry(np.pi), q_step2_1) 
        q_step6_1  = q_step2_1
        q_step7    = q_step5
        q_step7_1  = q_step2_1                                                                                                
        q_step8    = self.inverse_kinematics(SE3(pineapple_cord[0] + 0.00, pineapple_cord[1] + 0.00, pineapple_cord[2] + 0.1 + 0.026) @ SE3.Ry(np.pi), q_step2_1) 
        q_step8_1  = q_step2_1
        q_step9    = q_step5
        q_step9_1  = q_step2_1
        q_step10   = self.inverse_kinematics(SE3(cheese_cord[0]    + 0.00, cheese_cord[1]    + 0.00, cheese_cord[2]    + 0.1 + 0.016) @ SE3.Ry(np.pi), q_step2_1) 
        q_step10_1 = q_step2_1
        q_step11   = q_step5
        q_step12   = q_step1
        
        # Calculate trajectories
        q_traj1    = rtb.jtraj(   q_step1,    q_step2,  30).q
        q_traj2    = rtb.jtraj(   q_step2,  q_step2_1,  10).q
        q_traj2_1  = rtb.jtraj( q_step2_1,    q_step3,  10).q
        q_traj3    = rtb.jtraj(   q_step3,  q_step3_1,  10).q
        q_traj3_1  = rtb.jtraj( q_step3_1,    q_step4,  10).q
        q_traj4    = rtb.jtraj(   q_step4,  q_step4_1,  10).q
        q_traj4_1  = rtb.jtraj( q_step4_1,    q_step5,  10).q
        q_traj5    = rtb.jtraj(   q_step5,  q_step5_1,  10).q
        q_traj5_1  = rtb.jtraj( q_step5_1,    q_step6,  10).q
        q_traj6    = rtb.jtraj(   q_step6,  q_step6_1,  10).q
        q_traj6_1  = rtb.jtraj( q_step6_1,    q_step7,  10).q
        q_traj7    = rtb.jtraj(   q_step7,  q_step7_1,  10).q
        q_traj7_1  = rtb.jtraj( q_step7_1,    q_step8,  10).q
        q_traj8    = rtb.jtraj(   q_step8,  q_step8_1,  10).q
        q_traj8_1  = rtb.jtraj( q_step8_1,    q_step9,  10).q
        q_traj9    = rtb.jtraj(   q_step9,  q_step9_1,  10).q
        q_traj9_1  = rtb.jtraj( q_step9_1,   q_step10,  10).q
        q_traj10   = rtb.jtraj(  q_step10, q_step10_1,  10).q
        q_traj10_1 = rtb.jtraj(q_step10_1,   q_step11,  10).q
        q_traj11   = rtb.jtraj(  q_step11,   q_step12,  30).q
        
        # Join togther trajectories
        q_traj_final = np.concatenate([
            q_traj1  ,
            q_traj2  ,
            q_traj2_1,
            q_traj3  ,
            q_traj3_1,
            q_traj4  ,
            q_traj4_1,
            q_traj5  ,
            q_traj5_1,
            q_traj6  ,
            q_traj6_1,
            q_traj7  ,
            q_traj7_1,
            q_traj8  ,
            q_traj8_1,
            q_traj9  ,
            q_traj9_1,
            q_traj10 ,
            q_traj10_1,
            q_traj11 
        ], axis=0)
        
        fkine_result = [self.robot.fkine(q) @ SE3.Tz(0.1) for q in q_traj_final]
        
        return fkine_result, q_traj_final # (30*2)+(10*18) = 240 steps

class Robot3Movement(MovementCalculation):
    """Controls Robot 3 (Oven handling robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
    
    def calculate(self):
        """
        pick up pizza
        place in oven
        wait outside
        pick up pizza
        place in box
        """
        pizza_cord = self.pizza.xyz_of_node()
        
        # Joint anges at each step
        q_step1 = self.robot.q  # initial configuration
        
        # q_step2 = self.inverse_kinematics(SE3(8.76,3.678,0.994) * SE3.RPY(1.442,-0.460,-1.628, order='xyz'), q_step1)
        # q_step3 = self.inverse_kinematics(SE3(10.27,4.998,1.109) * SE3.RPY(1.522,0.735,-1.538, order='xyz'), q_step2)
        # q_step4 = self.inverse_kinematics(SE3(10.36,6.206,1.663) * SE3.RPY(-1.475,0.842,1.585, order='xyz'), q_step3)
        # q_step5 = self.inverse_kinematics(SE3(11.38,7.191,1.130) * SE3.RPY(-1.455,0.761,1.429, order='xyz'), q_step4)
        # q_step6 = self.inverse_kinematics(SE3(7.571,6.288,0.968) * SE3.RPY(-1.408,-1.222,1.766, order='xyz'), q_step5)
        

        q_step2 = self.inverse_kinematics(SE3(8.8,3.7,1.0) * SE3.RPY( 1.570796, -0.785398, -1.570796, order='xyz'), q_step1)
        q_step3 = self.inverse_kinematics(SE3(10.3,5.0,1.1) * SE3.RPY( 1.570796,  0.785398, -1.570796, order='xyz'), q_step2)
        q_step4 = self.inverse_kinematics(SE3(10.4,6.2,1.7) * SE3.RPY(-1.570796,  0.785398,  1.570796, order='xyz'), q_step3)
        q_step5 = self.inverse_kinematics(SE3(11.4,7.2,1.1) * SE3.RPY(-1.570796,  0.785398,  1.570796, order='xyz'), q_step4)
        q_step6 = self.inverse_kinematics(SE3(7.5,6.5,1.0) * SE3.RPY(-1.570796, -1.570796,  1.570796, order='xyz'), q_step5)
        
        q_step7 = q_step1 # return to initial
        
        # Calculate trajectories
        q_traj1   = rtb.jtraj(q_step1, q_step2, 15).q
        q_traj2   = rtb.jtraj(q_step2, q_step3, 15).q
        q_traj3   = rtb.jtraj(q_step3, q_step4, 15).q
        q_traj4   = rtb.jtraj(q_step4, q_step5, 15).q
        q_traj5   = rtb.jtraj(q_step5, q_step6, 15).q
        q_traj6   = rtb.jtraj(q_step6, q_step7, 15).q

        
        q_traj_final = np.concatenate([q_traj1, q_traj2, q_traj3, q_traj4, q_traj5, q_traj6], axis=0)
        
        fkine_result = [self.robot.fkine(q) @ SE3.Rx(pi/2) for q in q_traj_final]
        
        return fkine_result, q_traj_final # (15*7) = 105 steps
        
        
class Robot4Movement(MovementCalculation):
    """Controls Robot 4 (Packaging and loading robot) movements and task execution."""
    def __init__(self, robot_model):
        super().__init__(robot_model)
    
    def calculate(self):
        """
        pick up pizza box
        place on motorcycle
        """
        q_step1 = self.robot.q
        q_step2 = self.inverse_kinematics(SE3(7.1, 6.5, 1.0) * SE3.RPY( 1.5708, -0.7854, -1.5708, order='xyz'), q_step1)
        q_step3 = self.inverse_kinematics(SE3(5.2, 7.2, 1.2) * SE3.RPY(-1.5708,  0.7854,  1.5708, order='xyz'), q_step2)
        q_step4 = q_step1

        q_traj1 = rtb.jtraj(q_step1, q_step2, 10).q
        q_traj2 = rtb.jtraj(q_step2, q_step3, 20).q
        q_traj3 = rtb.jtraj(q_step3, q_step4, 10).q

        q_traj_final = np.concatenate([q_traj1, q_traj2, q_traj3], axis=0)
        fkine_result = [self.robot.fkine(q)for q in q_traj_final]

        return fkine_result, q_traj_final # (10*2) + 20 = 40 steps




    
if __name__ == "__main__":  # I sugest that you pause and zoom out alot to actualy see this test
    
    env = swift.Swift()
    env.launch(realtime=True)
    