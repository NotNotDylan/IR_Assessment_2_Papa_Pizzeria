import os
from typing import Optional, List
from pathlib import Path

import swift
from ir_support import UR3

from spatialgeometry import Mesh
from spatialmath import SE3

class ObjectNode:
    """
    Minimal scene-graph node for Swift/SpatialGeometry meshes with parent-child attachments.
    - Absolute pose lives in self.mesh.pose (SE3)
    - If attached: self._T_parent_this stores the child's pose relative to parent (SE3)
    """

    """def __init__(self, env: swift, pose: SE3, stl_path: str, color=(0.6, 0.6, 0.6, 1.0), name: Optional[str] = None):

        self.env = env

        if not isinstance(pose, SE3):
            raise TypeError("pose must be an SE3")
        if not os.path.isfile(stl_path):
            raise FileNotFoundError(f"STL not found: {stl_path}")

        self.name = name or os.path.basename(stl_path)
        self.mesh = Mesh(filename=stl_path, color=color, pose=pose)

        self.parent: Optional['ObjectNode'] = None
        self.children: List['ObjectNode'] = []
        # Pose of THIS node relative to parent (parent -> this). Identity if no parent.
        self._T_parent_this: SE3 = SE3()"""
        
    def __init__(
            self,
            env: swift,
            pose: SE3,
            stl_path: str | Path,
            color=(0.6, 0.6, 0.6, 1.0),
            name: Optional[str] = None,
            asset_root: Path | None = None,   # NEW: base dir for relative paths
        ):
        self.env = env
        if not isinstance(pose, SE3):
            raise TypeError("pose must be an SE3")

        # Resolve to an absolute, OS-safe path
        p = Path(stl_path)
        if asset_root is None:
            # default to the project directory that contains this file
            asset_root = Path(__file__).resolve().parent
        if not p.is_absolute():
            p = (asset_root / p).resolve()

        if not p.is_file():
            raise FileNotFoundError(
                f"STL not found: {p}\n"
                f"  CWD: {Path.cwd()}\n"
                f"  asset_root: {asset_root}"
            )

        self.name = name or p.stem
        self.mesh = Mesh(filename=str(p), color=color, pose=pose)
        self.parent: Optional['ObjectNode'] = None
        self.children: List['ObjectNode'] = []
        # Pose of THIS node relative to parent (parent -> this). Identity if no parent.
        self._T_parent_this: SE3 = SE3()

    # ----------- Convenience accessors -----------
    @property
    def pose(self) -> SE3:
        # mesh.T may be an ndarray or SE3 depending on version; normalize to SE3
        T = self.mesh.T
        return T if isinstance(T, SE3) else SE3(T)

    @pose.setter
    def pose(self, new_pose: SE3):
        if not isinstance(new_pose, SE3):
            raise TypeError("new_pose must be an SE3")
        # Some versions accept SE3 directly; others want a 4x4
        try:
            self.mesh.T = new_pose
        except Exception:
            self.mesh.T = new_pose.A
    
    def xyz_of_node(self):
        """Return the node's Cartesian position.

        Returns:
            numpy.ndarray | tuple[float, float, float]: The translation in (x, y, z) order.
                If a NumPy array, it is a length-3 vector with shape (3,).
        """
        return self.pose.t

    # ----------- Core operations -----------
    def set_pose(self, new_pose: SE3, propagate: bool = True) -> None:
        """Set absolute/world pose. If attached, update stored relative transform. Then propagate to children."""
        if not isinstance(new_pose, SE3):
            raise TypeError("new_pose must be an SE3")

        self.pose = new_pose

        # If this node has a parent, keep the stored relative transform consistent with the new absolute pose.
        if self.parent is not None:
            self._T_parent_this = self.parent.pose.inv() * self.pose

        if propagate:
            self._propagate_to_descendants()

    def move_by(self, delta: SE3) -> None:
        """Apply a delta transform in world coordinates."""
        if not isinstance(delta, SE3):
            raise TypeError("delta must be an SE3")
        self.set_pose(self.pose * delta, propagate=True)

    def light_screen_e_stop(self):
        x,y,z = self.xyz_of_node()
        if 3.10 <= x <= 7.32 and 4.725 <= y <= 4.85 and 0 <= z <=3:
            print("collision")
            self.move_by(SE3(0,0.1,0))
        else:
            self.move_by(SE3(0,0.1,0))



    def attach_to(self, parent: 'ObjectNode', keep_world_pose: bool = True) -> None:
        """
        Attach this node under `parent`.
        If keep_world_pose=True, the part does not jump: we compute and store the current relative transform.
        If keep_world_pose=False, the part snaps to the parent's pose (relative = identity).
        """
        if parent is self:
            raise ValueError("Cannot attach a node to itself.")
        if parent.is_descendant_of(self):
            raise ValueError("Cannot create a cycle: parent is a descendant of this node.")

        # Remove from previous parent if any
        if self.parent is not None:
            self.parent.children.remove(self)

        self.parent = parent
        parent.children.append(self)

        if keep_world_pose:
            # Preserve current absolute pose; just record the relation to parent
            self._T_parent_this = parent.pose.inv() * self.pose
            # Ensure descendants get consistent refresh (no visible jump)
            self._propagate_to_descendants()
        else:
            # Snap to parent's frame
            self._T_parent_this = SE3()
            self.set_pose(parent.pose * self._T_parent_this, propagate=True)

    def detach(self) -> None:
        """Detach from current parent; absolute pose stays the same."""
        if self.parent is not None:
            self.parent.children.remove(self)
            self.parent = None
            self._T_parent_this = SE3()  # identity when free-floating

    def set_local_to_parent(self, T_parent_this: SE3) -> None:
        """
        Directly set the relative pose to the parent, then update absolute pose accordingly.
        Useful when you want to edit the child's offset around its parent.
        """
        if self.parent is None:
            raise RuntimeError("set_local_to_parent called but node has no parent.")
        if not isinstance(T_parent_this, SE3):
            raise TypeError("T_parent_this must be an SE3")

        self._T_parent_this = T_parent_this
        self.set_pose(self.parent.pose * self._T_parent_this, propagate=True)

    # ----------- World/Swift helpers -----------
    def add_to_world(self) -> None:
        """world should have .env.add(mesh) like a Swift world wrapper."""
        self.env.add(self.mesh)

    # If your env supports removal; otherwise omit.
    def remove_from_world(self) -> None:
        try:
            self.env.remove(self.mesh)
        except Exception:
            pass

    # ----------- Internals -----------
    def _propagate_to_descendants(self) -> None:
        """Recompute absolute poses for all descendants from stored relative transforms."""
        for child in self.children:
            child.pose = self.pose * child._T_parent_this
            child._propagate_to_descendants()

    def is_descendant_of(self, other: 'ObjectNode') -> bool:
        p = self.parent
        while p is not None:
            if p is other:
                return True
            p = p.parent
        return False

    def __repr__(self) -> str:
        return f"ObjectNode(name={self.name!r}, children={len(self.children)}, attached={self.parent is not None})"


if __name__ == "__main__":  # I sugest that you pause and zoom out alot to actualy see this test
    
    env = swift.Swift()
    env.launch(realtime=True)
    
    base = ObjectNode(env, SE3(0.0, 0.0, 0.0), "IRB_4600/Base.stl", color=(0.5,0.5,0.5,1.0), name="PillarA")
    arm  = ObjectNode(env, SE3(0.6, 0.4, 0.2), "IRB_4600/Link_1.stl",    color=(0.8,0.2,0.2,1.0), name="ArmOnA")
    bird = ObjectNode(env, SE3(0,0,0), "Environment/Bird.stl", color=(0,0,0))

    base.add_to_world()
    arm.add_to_world()
    bird.add_to_world()

    # Attach the arm to the pillar, preserving its current placement
    arm.attach_to(base, keep_world_pose=True)

    for i in range(25):
        # Move the pillar — arm follows with its saved relative offset
        base.move_by(SE3.Tx(0.025 * i) * SE3.Rz(0.035))
        env.step(0.05)

    for i in range(25):
        # You can also reposition the arm directly; its relative transform updates automatically
        arm.set_pose(SE3(0.010 * i, 0.050 * i, 0.30) * SE3.Rz(0.01 * i))
        env.step(0.05)
        
    for i in range(25):
        # Move the pillar — arm follows with its saved relative offset
        base.move_by(SE3.Tx(0.025 * i) * SE3.Rz(0.035))
        env.step(0.05)