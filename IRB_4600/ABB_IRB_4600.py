import os
import time
from math import pi

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from spatialgeometry import Arrow, Cuboid, Cylinder
import numpy as np
from scipy import linalg


from ir_support.robots.DHRobot3D import DHRobot3D
from roboticstoolbox import DHLink, DHRobot, models, jtraj, trapezoidal
from ir_support import RectangularPrism, line_plane_intersection, CylindricalDHRobotPlot

from ir_support import EllipsoidRobot
import trimesh

    
def deg2rad(degrees):
    """Convert degrees to radians."""
    return degrees * pi / 180


class IRB_4600(DHRobot3D):
    """
    ABB IRB 2400 (payload 10 kg or 16 kg) built as a DH model with 3D link geometry.

    Parameters
    ----------
    variant : str
        "2400_10" or "2400_16" (kinematics are the same; payload differs)
    """

    # ------------------------------
    def __init__(self, variant: str = "2400_16"):
        links = self._create_DH()
        self.variant = variant
         
        # File basenames for your link models (no extension here; DHRobot3D will try .dae/.stl)
        link3D_names = dict(
            link0="Base",       
            link1="Link_1", 
            link2="Link_2", 
            link3="Link_3",  
            link4="Link_4",   
            link5="Link_5",   
            link6="Link_6_V2",
        )

        # A convenient "inspection" configuration to align meshes
        qtest = [0, 0, 0, 0, 0, 0]

        # Aligning 3D meshes to the DH frames at qtest.        
        qtest_transforms = [
            spb.transl(0, 0, 0),                                     # link0: base casting
            spb.transl(0.0, 0.002917   , 0.1595),    # link1: shoulder
            spb.transl(0.17195, -0.1225, 0.495)  @ spb.trotx(pi/2),    # link2: upper arm
            spb.transl(0.17301, -0.1492     , 1.59)  @ spb.trotx(-pi/2),    # link3: forearm
            spb.transl(0.5151, 0.0, 1.765)  @ spb.troty( pi/2),    # wrist1
            spb.transl(1.4055, -0.0205     , 1.765)  @ spb.trotx(-pi/2),    # wrist2
            spb.transl(1.4565 , 0.0     , 1.765)  @ spb.troty( pi/2),    # wrist3 (flange)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links,
            link3D_names,
            name=f"IRB4600_{variant}",
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )


        self.q = qtest

    # ------------------------------
    def _create_DH(self):
        """
        Create standard DH model.
        """
        # --- Derived values from your CAD ---
        # axis   d (m)=displacment along z     a (m)=length between     alpha (rad)=rot around x     qlim (rad)
        a      = [0.175,  1.095, 0.175,   0.0,   0.0,  0.000 ]
        d      = [0.495, 0.00,  0.0,   1.2305, 0.00,  0.085]
        alpha  = [-pi/2,  0.0,   -pi/2,   +pi/2,  +pi/2, 0.0 ]  
        offset = [    0,  -pi/2,     0,   0,      pi,     0 ]
        
        
        # Axis ranges from ABB spec (convert degrees to radians)
        qlim = [
            [-2*pi, +2*pi],                            # A1 ±180°
            [-0.9,1.6],                              # A2 -100..+110
            [-1.2, 1.3],                            # A3 -60..+65
            [-pi, pi],        # A4 ±200 (unlimited opt.)
            [deg2rad(-125), deg2rad(+120)],        # A5 ±120
            [-pi, pi],        # A6 ±400
        ]
        
        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i],))      
        return links
    
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


    def test(self):
        """
        Visual test in Swift: generate a trajectory and replan if it intersects
        the forbidden zone, then animate the successful one.
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.add_to_env(env)
        env.set_camera_pose([2.5, -2.0, 1.5], [0, 0, 1.0])

        # Add a transparent cube showing the restricted zone
        forbidden_zone = Cuboid(
            scale=[1, 1, 0.5],
            base=SE3(1, 1, 1.25),
            color=(1, 0, 0, 0.3),  # semi-transparent red
        )
        env.add(forbidden_zone)

        # Define start and target poses
        start_q = self.q.copy()
        target_pose = SE3(1.5 , 1.5, 0.5) * SE3.Rz(0)  # reachable location above the zone

        steps = 150
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
            env.step(0.05)

        
        # Early tests that don't matter so much now

        # q = self.q
        # T1 = self.fkine(q)
        # x1 = T1.t


        # T2 = SE3(1,1,0.250)
        # x2 = T2.t

        # delta_t = 0.05
        # steps = 100
        

        # box = trimesh.load(r'C:\Users\Aidan\Desktop\PyIR\Assignment 2\Collision_Test.stl')

        # inside = self.is_point_inside_mesh(box,[1, 1, 0.25])
        # print(inside)
        # print(box)
        # print("box:", box)
        # print("bounds:", box.bounds)           # min and max extents
        # print("centroid:", box.centroid)       # mesh centroid
        # print("is_watertight:", box.is_watertight)
        # print("face_normals valid:", box.face_normals.shape)



        # # prism = Cuboid(scale=(1,1,0.4), color=[0.0, 1.0, 0.0, 0.5],pose=SE3(1,1,0.2))
        # # env.add(prism)

        # x = np.zeros([3,steps])
        # s = trapezoidal(0,1,steps).q
        # for i in range(steps):
        #     x[:,i] = x1*(1-s[i]) + s[i]*x2
        
        # q_matrix = np.zeros([steps, 6])

        # q_matrix[0,:] = self.ikine_LM(T1).q

        # Thing = SE3(3,5,6) @ SE3.Rx(pi/2) @ SE3.Ry(0) @ SE3.Rz(0)
        # roll, pitch, yaw = Thing.rpy(unit="rad", order="zyx")

        # for i in range(steps-1):                     # Calculate velocity at discrete time step
        #     J = self.jacob0(q_matrix[i, :])  # 6x6 full Jacobian
        #     xdot_linear = (x[:, i+1] - x[:, i]) / delta_t  # 3x1
        #     xdot_angular = np.array([roll/(steps*delta_t), pitch/(steps*delta_t), yaw/(steps*delta_t)])                # keep orientation fixed (pi)/(steps*delta_t)
        #     xdot_full = np.hstack((xdot_linear, xdot_angular))
        #     #print(xdot_full)  # 6x1
        #     q_dot = np.linalg.pinv(J) @ xdot_full
        #     q_matrix[i+1,:] = q_matrix[i,:] + delta_t * q_dot

        # points = []  # collect all FK positions


        # for q in q_matrix:
        #     self.q = q
        #     S = self.fkine_path(q)    
        #     for T in S:     
        #         point = T.t
        #         points.append(point)
        #     env.step(0.05) 

        # # Convert to NumPy array of shape (n,3)
        # points_array = np.array(points)
        # for i in range(len(points_array)):
        #     if -2 <= points_array[i][0] <= 2:
        #         if -2 <= points_array[i][1] <= 2:
        #             if 0 <= points_array[i][2] <= 5:
        #                 print("COLLISION at", points_array[i])
        # Check which points are inside the box
        

        # Print results



if __name__ == "__main__":    
    IRB_4600().test()