from spatialmath import SE3
from spatialgeometry import Mesh
import os
import sys
import numpy as np
from typing import Optional

# ensure the parent package/directory is on sys.path so local modules can be imported
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from manipulatable_object import ObjectNode
import swift
import time

# TODO: Attach it to a robot
# TODO: Make it specilised for my robot, that will alow me to neaten the whole thing up a heap and make the code way more practical

class IRB_2400_Gripper(ObjectNode):
    def __init__(
        self,
        env,
        robot,
        name: Optional[str] = "GripperBase"
    ):
        
        # locate assets relative to this file (the .stl files live in the same folder as this script)
        base_dir = os.path.dirname(__file__)
        base_stl = os.path.join(base_dir, "Gripper_basev2.stl")
        left_claw_stl = os.path.join(base_dir, "Gripper_fingerv2.stl")
        right_claw_stl = os.path.join(base_dir, "Gripper_fingerv2.stl")
        
        base_pose=SE3(0.0, 0.0, 0.0)
        self.robot = robot
        self.env = env
        
        super().__init__(env, base_pose, base_stl, name=name)

        # Store transform parameters
        self.left_offset = SE3.Tx(0.0)
        self.right_offset = SE3.Ty(0.26)
        self.rotation = SE3.Rx(0)
        self.open_dist = 0.00
        self.closed_dist = -0.02

        # Gripper claws as ObjectNodes
        self.left_claw = ObjectNode(env, base_pose, left_claw_stl, name="LeftClaw")
        self.right_claw = ObjectNode(env, base_pose, right_claw_stl, name="RightClaw")

        self.left_claw.attach_to(self)
        self.right_claw.attach_to(self)

        self.left_claw.add_to_world()
        self.right_claw.add_to_world()
        self.add_to_world()

        self._is_closed = False
        self.gripped_object = None

        self.update_claws(self.open_dist)

    def update_claws(self, width: float):
        """Update claw offsets based on desired width."""
        left_pose = SE3.Ty(-width) * self.left_offset * self.rotation
        right_pose = SE3.Ty(width) * self.right_offset * self.rotation

        self.left_claw.set_local_to_parent(left_pose)
        self.right_claw.set_local_to_parent(right_pose)

    def close(self, obj: Optional[ObjectNode] = None):
        self.update_claws(self.closed_dist)
        self._is_closed = True

        if obj is not None:
            obj.attach_to(self)
            self.gripped_object = obj

    def open(self):
        self.update_claws(self.open_dist)
        self._is_closed = False

        if self.gripped_object is not None:
            self.gripped_object.detach()
            self.gripped_object = None

    def is_closed(self):
        return self._is_closed

    def is_open(self):
        return not self._is_closed

def test():
    # Just using a random stl as an object to move
    box_stl = os.path.join(os.path.dirname(__file__), "Gripper_basev2.stl") 
    
    # create a simple box to pick (if mesh exists)
    box = None
    if os.path.exists(box_stl):
        box = ObjectNode(env, SE3(0.3, 0.0, 0.02), box_stl, name="Box")
        box.add_to_world()
    else:
        print("Box mesh missing: skipping attach-demo. Gripper demo will still show movement.")

    # Helper to step the simulator with a small pause for visualization
    def step(n=1, dt=0.05):
        for _ in range(n):
            env.step(dt)
            time.sleep(0.01)

    # Start with gripper open and visually centered above the box
    gripper.open()
    step(10)

    # Approach: lower gripper in small increments
    for _ in range(40):
        gripper.move_by(SE3.Tz(-0.002))  # move down by 2 mm
        step(1)

    # If we have a box, align it under the gripper and close to attach
    if box is not None:
        # ensure the box is roughly below the jaws, then attach via close(obj=...)
        gripper.close(obj=box)
    else:
        gripper.close()

    step(25)

    # Lift the object up
    for _ in range(50):
        gripper.move_by(SE3.Tz(0.002))  # move up by 2 mm
        step(1)

    step(10)

    # Release the object
    gripper.open()
    step(20)

    print("Demo finished.")

if __name__ == "__main__":

    env = swift.Swift()
    env.launch()

    # create the gripper (robot can be None for this demo)
    gripper = IRB_2400_Gripper(
        env=env,
        robot=None,
        name="DemoGripper",
    )
    
    test(gripper)