from pathlib import Path
import os, time
from math import pi
import numpy as np

# ---- Swift path setup swift root is cooked on pc have to manually direct to correct root address ----
SCRIPT_DIR = Path(__file__).parent.resolve()
SWIFT_ROOT = Path(SCRIPT_DIR.drive + "/").resolve()
os.chdir(SWIFT_ROOT)

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D

def deg2rad(degrees): return degrees * pi / 180.0

# --- root address shower
def swift_prefix_from_windows_path(p: Path) -> str:
    """Turn 'C:/foo/bar' into 'foo/bar/' for Swift /retrieve/."""
    s = p.as_posix()
    if ":" in s: s = s.split(":", 1)[1]   
    s = s.lstrip("/")                     
    return s + ("/" if not s.endswith("/") else "")

# ---------- sweep helpers ----------
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

def sweep_joint(env, robot, j, q_hold=None, lo=None, hi=None,
                steps=240, dt=0.01, cycles=1, echo_every=40):
    q = np.array(robot.q if q_hold is None else q_hold, dtype=float)

    if lo is None or hi is None:
        lo, hi = get_limits(robot, j)

    onepass = np.concatenate([
        np.linspace(lo, hi, steps),
        np.linspace(hi, lo, steps)
    ])
    path = np.tile(onepass, cycles)



    for k, s in enumerate(path, 1):
        q[j] = s
        robot.q = q
        env.step(dt)
        if echo_every and (k % echo_every == 0 or k == len(path)):
            print(f"  J{j+1} = {robot.q[j]:+.4f} rad  |  q = {np.round(robot.q, 4)}")



def sweep_all(env, robot, dwell=0.5, **kwargs):
    """
    Sweep each joint sequentially while holding others fixed at the start pose (robot.q).
    kwargs pass to sweep_joint (steps, dt, cycles, lo/hi, echo_every).
    """
    q_home = np.array(robot.q, dtype=float)
    for j in range(robot.n):
        print(f"\n=== Sweeping J{j+1} ===")
        sweep_joint(env, robot, j, q_hold=q_home, **kwargs)
        time.sleep(dwell)


# ---------- robot ----------
class AuboI5(DHRobot3D):
    def __init__(self, variant: str = "2400_16"):
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

        # world poses
        qtest_transforms = [
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0.05) @spb.trotz(pi),                           # link1
            spb.transl(0.0, 0.069, 0.1285),  # link2
            spb.transl(0.0, 0.069, 0.5365) @spb.trotz(pi),                 # link3
            spb.transl(0.0, 0.0795, 0.9125) @ spb.trotz(pi), # link4
            spb.transl(0.0, 0.1275, 0.973),                  # link5
            spb.transl(0.0, 0.1874, 1.021) @spb.trotz(pi),  # link6 (tool)
        ]

        link3d_dir = swift_prefix_from_windows_path(SCRIPT_DIR)

        super().__init__(
            links,
            link3D_names,
            name=f"Aubo I5{variant}",
            link3d_dir=link3d_dir,
            qtest=qtest,
            qtest_transforms=qtest_transforms,
        )

        self.q = qtest

    def _create_DH(self):
        a      = [0.0,  0.00,   0.41,  0.38, 0.00, 0.0]
        d      = [0.123,   0.5,  0.01, -0.383,  0.11,  0.0]
        alpha  = [0, -pi/2,  0.0, 0, -pi/2, pi/2]
        offset = [0.0,  0.0,  0.0, 0.0,  0.0,   0.0]

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


    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)
        self.base = SE3()
        self.plot(self.q)

        self.add_to_env(env)

        #run the sweeps
        sweep_all(env, self, steps=220, dt=0.01, cycles=1, dwell=0.35, echo_every=40)

        env.hold()


if __name__ == "__main__":
    AuboI5().test()
