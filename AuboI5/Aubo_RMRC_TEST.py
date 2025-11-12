from pathlib import Path
import os
from math import pi
import numpy as np

# ---- Swift path setup: make Swift root the drive root so /retrieve/ works ----
SCRIPT_DIR = Path(__file__).parent.resolve()
SWIFT_ROOT = Path(SCRIPT_DIR.drive + "/").resolve()
os.chdir(SWIFT_ROOT)

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D

# Optional: trimesh collision check (only used if you pass a box path)
try:
    import trimesh
except Exception:
    trimesh = None


def deg2rad(degrees): 
    return degrees * pi / 180.0


def swift_prefix_from_windows_path(p: Path) -> str:
    """Turn 'C:/foo/bar' into 'foo/bar/' for Swift /retrieve/."""
    s = p.as_posix()
    if ":" in s:
        s = s.split(":", 1)[1]
    s = s.lstrip("/")
    return s + ("/" if not s.endswith("/") else "")


# Keep only this small helper the RMRC uses
def get_limits(robot, j, default=(-pi, pi)):
    try:
        lo, hi = float(robot.qlim[0, j]), float(robot.qlim[1, j])
        if np.isfinite(lo) and np.isfinite(hi):
            if lo > hi:
                lo, hi = hi, lo
            return lo, hi
    except Exception:
        pass
    return default


class AuboI5(DHRobot3D):
    def __init__(self, variant: str = "i5"):
        links = self._create_DH()
        self.variant = variant

        link3D_names = dict(
            link0="L0",
            link1="L1V1",
            link2="L2V1",
            link3="L3V1",
            link4="L4V1",
            link5="L5V1",
            link6="L6V1",
        )

        # inspection config
        qtest = [0, -pi/2, 0, -pi/2, 0, 0]

        # world poses (mesh-to-DH alignment at qtest)
        qtest_transforms = [
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0.05) @ spb.trotz(pi),                 # link1
            spb.transl(0.0, 0.069, 0.1285),                         # link2
            spb.transl(0.0, 0.069, 0.5365) @ spb.trotz(pi),         # link3
            spb.transl(0.0, 0.0795, 0.9125) @ spb.trotz(pi),        # link4
            spb.transl(0.0, 0.1275, 0.973),                         # link5
            spb.transl(0.0, 0.1874, 1.021) @ spb.trotz(pi),         # link6 (tool)
        ]

        link3d_dir = swift_prefix_from_windows_path(SCRIPT_DIR)

        super().__init__(
            links,
            link3D_names,
            name=f"Aubo I5 {variant}",
            link3d_dir=link3d_dir,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.q = qtest

    def _create_DH(self):
        a      = [0.0,   0.00,  0.41,   0.38, 0.00, 0.0]
        d      = [0.123, 0.50,  0.01,  -0.383, 0.11, 0.0]
        alpha  = [0,    -pi/2,  0.0,    0,   -pi/2, pi/2]
        offset = [0.0,   0.0,   0.0,    0.0,  0.0,  0.0]

        qlim = [
            [deg2rad(-360), deg2rad(+360)],
            [deg2rad(-175), deg2rad(+175)],
            [deg2rad(-162), deg2rad(+162)],
            [deg2rad(-175), deg2rad(+175)],
            [deg2rad(-175), deg2rad(+175)],
            [deg2rad(-360), deg2rad(+360)],
        ]

        return [
            rtb.RevoluteMDH(d=d[i], a=a[i], alpha=alpha[i],
                            offset=offset[i], qlim=qlim[i]) for i in range(6)
        ]

    # ------------------------------
    # RMRC DEMO
    def rmrc_demo(self, T_goal: SE3 | None = None, steps = 140):
        dt = 0.05
        camera_pose=((1.8, -1.8, 1.2), (0, 0, 0.8))
        box_stl_path = None
        damping = 2e-3
        """
        Run a simple RMRC move from current EE pose to T_goal.
        - Damped least squares for robustness near singularities.
        - Optional constant-orientation or ZYX rpy interpolation.
        - Optional trimesh box inclusion test (if you give an STL path).
        """
        # --- environment ---
        env = swift.Swift()
        env.launch(realtime=True)
        self.add_to_env(env)

        env.set_camera_pose(*camera_pose)

        # --- start/goal poses ---
        q0 = np.asarray(self.q, dtype=float)
        T1 = self.fkine(q0)
        if T_goal is None:
            T_goal = SE3(x1[0] + 0.25, x1[1] + 0.15, max(0.1, x1[2] + 0.10))
        x1 = T1.t
        x2 = T_goal.t

        # orientation interpolation (ZYX)
        # rpy1 = np.array(T1.rpy(unit="rad", order="zyx"))
        # rpy2 = np.array(T_goal.rpy(unit="rad", order="zyx"))
        ang_rate = np.zeros(3)

        # --- task-space path (trapezoidal scalar blend) ---
        s = rtb.trapezoidal(0, 1, steps).q                 # (steps,)
        X = x1[:, None] * (1 - s) + x2[:, None] * s        # (3, steps)

        # --- joint limits (vectorized clamp) ---
        q_lo = np.array([get_limits(self, j)[0] for j in range(self.n)])
        q_hi = np.array([get_limits(self, j)[1] for j in range(self.n)])

        # --- init path ---
        q = np.empty((steps, self.n))
        q[0] = np.clip(q0, q_lo, q_hi)

        # --- integrate RMRC + animate (single pass) ---
        ee_points = [T1.t]
        for i in range(steps - 1):
            J = self.jacob0(q[i])  # 6x6

            # damped pseudoinverse: J^T (J J^T + λ^2 I)^-1
            JJt = J @ J.T
            pinv = J.T @ np.linalg.inv(JJt + (damping ** 2) * np.eye(J.shape[0]))

            xdot_lin = (X[:, i + 1] - X[:, i]) / dt
            xdot = np.hstack((xdot_lin, ang_rate))  # (6,)

            q[i + 1] = np.clip(q[i] + dt * (pinv @ xdot), q_lo, q_hi)

            # animate & record EE
            self.q = q[i + 1]
            ee_points.append(self.fkine(self.q).t)
            env.step(dt)

        # # --- optional mesh inclusion check for EE path ---
        # if box_stl_path and trimesh is not None and os.path.exists(box_stl_path):
        #     box = trimesh.load(box_stl_path)
        #     pts = np.vstack(ee_points)
        #     inside_mask = box.contains(pts)
        #     if np.any(inside_mask):
        #         coll_pts = pts[inside_mask]
        #         print(f"[RMRC] {coll_pts.shape[0]} points inside mesh; first few:\n{coll_pts[:5]}")
        #     else:
        #         print("[RMRC] No points inside the provided mesh.")
        # elif box_stl_path:
        #     print("[RMRC] trimesh not available or STL path not found; skipping mesh check.")
        # env.hold()
if __name__ == "__main__":
    robot = AuboI5()
    # Run RMRC demo (edit goal / options as needed)
    T_goal = SE3(0.4, 0.25, 0.35) @ SE3.Rx(pi/2)  # position + orientation target
    robot.rmrc_demo(T_goal = T_goal, steps=140)
