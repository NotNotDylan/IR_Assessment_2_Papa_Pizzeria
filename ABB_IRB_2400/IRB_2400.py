"""
@file
@brief ABB IRB 2400 DH-based robot (with 3D link models) for Swift/Robotics Toolbox
@author You
@date Oct 2025

Usage:
>>> from IRB2400 import IRB2400
>>> import swift
>>> r = IRB2400(variant="2400_16")  # or "2400_10"
>>> env = swift.Swift()
>>> env.launch(realtime=True)
>>> r.add_to_env(env)
>>> # quick move:
>>> import roboticstoolbox as rtb, numpy as np
>>> q_goal = np.array(r.q) + 0.1
>>> for q in rtb.jtraj(r.q, q_goal, 50).q:
...     r.q = q
...     env.step(0.02)
"""

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


class IRB2400(DHRobot3D):
    """
    ABB IRB 2400 (payload 10 kg or 16 kg) built as a DH model with 3D link geometry.

    Parameters
    ----------
    variant : str
        "2400_10" or "2400_16" (kinematics are the same; payload differs)
    mounting : str
        "floor" or "inverted" (geometry same, base transform differs)
    """

    # ------------------------------
    def __init__(self, variant: str = "2400_16"):
        links = self._create_DH()
        self.variant = variant


        # Un-edited = masive stl in m
        # edited = scaled correctly (from m to mm)
        # editedv2 = origen moved to aproprate location
        # editedv3 = z axis aligned with rotation
         
        # File basenames for your link models (no extension here; DHRobot3D will try .dae/.stl)
        link3D_names = dict(
            link0="base_irb2400_edited",       # Notes for how I rotated the model for editedv3
            link1="shoulder_irb2400_editedv3", # x 90
            link2="upperarm_irb2400_editedv3", # x 90
            link3="forearm_irb2400_editedv3",  # y -90
            link4="wrist1_irb2400_editedv3",   # x -90
            link5="wrist2_irb2400_editedv3",   # y -90
            link6="wrist3_irb2400_editedv3",   # y -90
        )

        # A convenient "inspection" configuration to align meshes (“zero-like” with elbow down)
        qtest = [0, -pi / 2, +pi / 2, 0, 0, 0]

        # INITIAL GUESSES for aligning 3D meshes to the DH frames at qtest.
        # Start simple: place each mesh near its joint frame; refine interactively later.
        # You will tune these using the steps in the guide below.        
        qtest_transforms = [
            spb.transl(0, 0, 0),                         # link0: base casting
            spb.transl(0.097, -0.1   , 0.615) @ spb.trotx(-pi/2),        # link1: shoulder
            spb.transl(0.0705, 0.1, 1.32)  @ spb.trotx(-pi/2),       # link2: upper arm
            spb.transl(0.364, 0     , 1.455) @ spb.troty(pi/2),       # link3: forearm
            spb.transl(0.855, 0.0205, 1.455) @ spb.trotx(pi/2),   # wrist1
            spb.transl(0.906, 0     , 1.455) @ spb.troty(pi/2),   # wrist2
            spb.transl(0.94 , 0     , 1.455) @ spb.troty(pi/2),       # wrist3 (flange)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))

        super().__init__(
            links,
            link3D_names,
            name=f"IRB2400_{variant}",
            link3d_dir=current_path,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        # Set starting pose (also applies inverted mounting if chosen)
        self.q = qtest

    # ------------------------------
    def _create_DH(self):
        """
        Create standard DH model.
        """
        # --- Derived values from your CAD ---
        # axis   d (m)=displacment along z     a (m)=length between     alpha (rad)=rot around x     qlim (rad)
        # a     = [ 0.097,  0.706,  0.135,  0.000,   0.000,  0.000 ]
        # d     = [ 0.615, -0.200,  0.100,  0.491,  -0.0205, 0.034 ]
        # alpha = [ +pi/2,  0.0,   +pi/2,  +pi/2,   +pi/2,   0.0  ]
        # Axis ranges from ABB spec (convert degrees to radians)
        
        a     = [ 0.100,  0.705, 0.135,  0.755 ,   0.000,  0.000 ]
        d     = [ 0.615,  0.0  ,   0.0,  0     ,  -0.0205, 0.034 ]
        alpha = [ +pi/2,    0.0, -pi/2,  +pi/2 ,   +pi/2,   0.0  ]  
           
        qlim = [
            [-pi, +pi],                            # A1 ±180°
            [deg2rad(-100), deg2rad(+110)],        # A2 -100..+110
            [deg2rad(-60),  deg2rad(+65)],         # A3 -60..+65
            [deg2rad(-200), deg2rad(+200)],        # A4 ±200 (unlimited opt.)
            [deg2rad(-120), deg2rad(+120)],        # A5 ±120
            [deg2rad(-400), deg2rad(+400)],        # A6 ±400
        ]

        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i]))
        return links

    # ------------------------------
    def test(self):
        """
        Smoke-test: add to Swift, jog a bit, then pause.
        """
        env = swift.Swift()
        env.launch(realtime=True)
        
        # create_minimal_axes(env)

        # Set a base offset so the robot isn't on the origin if you like
        # self.base = SE3(0.4, 0.4, 0) if self.mounting == "floor" else SE3(0.4, 0.4, 1.6) * SE3.Rx(pi)
        self.add_to_env(env)

        q_goal = [self.q[i] - pi/8 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 80).q
        for q in qtraj:
            self.q = q
            # fig = self.plot(self.q)
            env.step(0.02)
        time.sleep(2)
        # env.hold()
        # (use env.hold() interactively if needed)       




if __name__ == "__main__":    
    IRB2400().test()
