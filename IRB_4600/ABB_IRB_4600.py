import os
import time
from math import pi

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from spatialgeometry import Arrow

from ir_support.robots.DHRobot3D import DHRobot3D

 
    
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
            link6="Link_6",   
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
            spb.transl(1.3515, 0     , 1.765)  @ spb.trotx(-pi/2),    # wrist2
            spb.transl(1.4565 , 0     , 1.765)  @ spb.troty( pi/2),    # wrist3 (flange)
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
        a      = [ 0.175, 1.095, 0.175,   0.000, 0.000, 0.000 ]
        d      = [ 0.495,    0.02,   0.149, 1.2305,   0.0, 0.085 ]
        alpha  = [ -pi/2,    0.0, -pi/2, 0.0, +pi/2,   0.0 ]  
        offset = [     0,      -pi/2,     0,     0,     pi, 0 ]
        
        
        # Axis ranges from ABB spec (convert degrees to radians)
        qlim = [
            [-pi, +pi],                            # A1 ±180°
            [-pi, deg2rad(150)],                              # A2 -100..+110
            [deg2rad(-180), deg2rad(+75)],                            # A3 -60..+65
            [deg2rad(-400), deg2rad(+400)],        # A4 ±200 (unlimited opt.)
            [deg2rad(-125), deg2rad(+120)],        # A5 ±120
            [deg2rad(-400), deg2rad(+400)],        # A6 ±400
        ]

        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i]))
        return links

    # ------------------------------
    def test(self):
        """
        Smoke-test: add to Swift, jog a bit, then pause.
        """
        env = swift.Swift()
        env.launch(realtime=True)

        self.add_to_env(env)
        env.set_camera_pose([2.5, -2.0, 1.5], [0, 0, 1.0])

        q_goal = [self.q[i] - pi/8 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 20).q
        for q in qtraj:
            self.q = q
            fig = self.plot(self.q)
            env.step(0.02)
        time.sleep(2)
        # env.hold()     




if __name__ == "__main__":    
    IRB_4600().test()