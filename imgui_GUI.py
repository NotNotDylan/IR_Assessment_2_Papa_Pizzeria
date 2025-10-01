# """
# Immediate-mode GUI for Assessment 2 using Python Dear ImGui (pyimgui + GLFW).

# Install:
#   pip install imgui==2.* glfw PyOpenGL

# This GUI is *polled* from your main loop. Call `gui.tick()` every frame, then
# consume events via `gui.get_and_clear_events()`.

# It is intentionally stateless w.r.t. your robots/sim; you bind robots through
# a small adapter (get_q / set_q / qlim). No busy-waits; no blocking server.
# """

# from __future__ import annotations
# import math
# import time
# import threading
# from typing import Callable, Dict, List, Optional, Tuple

# import glfw  # window + input
# import OpenGL.GL as gl  # render backend
# import imgui
# from imgui.integrations.glfw import GlfwRenderer


# RobotId = int


# class GUIImGui:
#     """
#     Dear ImGui GUI for multi-robot control with latched e-stop.

#     Integration model:
#       - Call `start()` once after constructing.
#       - Call `tick()` each frame from your main loop (BEFORE swift env.step()).
#       - Pull UI intents via `get_and_clear_events()`; apply to your system.
#       - Call `shutdown()` on exit.

#     Events emitted (tuples):  (event_name, payload_dict_or_None)
#       - ('estop', {'source': 'gui'})
#       - ('estop_reset', None)                # user released e-stop latch
#       - ('resume', None)                     # explicit user resume
#       - ('stop_program', None)               # quit app
#       - ('select_robot', {'robot_id': int})
#       - ('set_q', {'robot_id': int, 'q': list[float]})
#       - ('jog_cart', {'robot_id': int, 'dx': float, 'dy': float, 'dz': float,
#                       'droll': float, 'dpitch': float, 'dyaw': float})

#     Bind robots via `bind_robots({...})` where each entry is:
#       {
#         'name': str,
#         'get_q': Callable[[], List[float]],
#         'set_q': Callable[[List[float]], None],   # you may ignore and consume via events instead
#         'qlim': List[Tuple[float, float]],        # per-joint (min,max), radians
#         'dof': int
#       }
#     """

#     # ---------- LIFECYCLE ----------
#     def __init__(self, title: str = "Industrial Robotics A2 — ImGui Panel", width: int = 440, height: int = 760):
#         self.title = title
#         self.width = width
#         self.height = height

#         # Window / renderer
#         self._window = None
#         self._impl: Optional[GlfwRenderer] = None

#         # External state inputs (provide from your sim each frame if desired)
#         self.light_curtain_broken: bool = False
#         self.system_paused: bool = False

#         # E-stop latch
#         self._estop_latched: bool = False

#         # Events buffer (producer = GUI, consumer = runner)
#         self._events: List[Tuple[str, Optional[dict]]] = []

#         # Robots registry
#         self._robots: Dict[RobotId, dict] = {}
#         self._active_robot: Optional[RobotId] = None

#         # UI cache per robot (slider values)
#         self._q_shadow: Dict[RobotId, List[float]] = {}

#         # Cartesian nudge increments (meters/radians)
#         self._nudge_lin = 0.005   # 5 mm nudge
#         self._nudge_ang = math.radians(2)  # 2 deg

#     def bind_robots(self, robots: Dict[RobotId, dict], default_robot: Optional[RobotId] = None):
#         """
#         Register robots. Can be called before or after start().
#         robots: { id -> {'name', 'get_q', 'set_q', 'qlim', 'dof'} }
#         """
#         self._robots = robots or {}
#         if default_robot and default_robot in self._robots:
#             self._active_robot = default_robot
#         elif self._robots and self._active_robot not in self._robots:
#             self._active_robot = sorted(self._robots.keys())[0]
#         # seed slider cache
#         for rid, meta in self._robots.items():
#             try:
#                 self._q_shadow[rid] = list(meta['get_q']())
#             except Exception:
#                 self._q_shadow[rid] = [0.0] * int(meta.get('dof', 6))

#     def start(self):
#         """Create window + renderer. Must be called from the main thread."""
#         if not glfw.init():
#             raise RuntimeError("glfw.init() failed")
#         glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
#         self._window = glfw.create_window(self.width, self.height, self.title, None, None)
#         if not self._window:
#             glfw.terminate()
#             raise RuntimeError("glfw.create_window() failed")

#         glfw.make_context_current(self._window)
#         imgui.create_context()
#         self._impl = GlfwRenderer(self._window, attach_callbacks=True)

#         # nicer fonts if you want: self._load_font('.../Inter.ttf', 16)

#     def shutdown(self):
#         """Destroy renderer + window."""
#         if self._impl:
#             self._impl.shutdown()
#             self._impl = None
#         if self._window:
#             glfw.destroy_window(self._window)
#             self._window = None
#         glfw.terminate()

#     # ---------- PUBLIC API TO FEED STATUS ----------
#     def set_light_curtain(self, broken: bool):
#         self.light_curtain_broken = bool(broken)

#     def set_system_paused(self, paused: bool):
#         """Tell the GUI whether the runner is currently paused (e.g., after e-stop)."""
#         self.system_paused = bool(paused)

#     # ---------- MAIN PER-FRAME TICK ----------
#     def tick(self):
#         """
#         Poll input, draw widgets, enqueue events. Call once per frame.
#         Safe to call even if window was closed (it will no-op after shutdown).
#         """
#         if not self._window or not self._impl:
#             return

#         if glfw.window_should_close(self._window):
#             self._events.append(('stop_program', None))
#             self.shutdown()
#             return

#         glfw.poll_events()
#         self._impl.process_inputs()

#         imgui.new_frame()
#         self._draw_main_panel()
#         imgui.render()

#         # Clear the framebuffer
#         gl.glViewport(0, 0, *glfw.get_framebuffer_size(self._window))
#         gl.glClearColor(0.08, 0.09, 0.10, 1.0)
#         gl.glClear(gl.GL_COLOR_BUFFER_BIT)
#         self._impl.render(imgui.get_draw_data())
#         glfw.swap_buffers(self._window)

#     def get_and_clear_events(self) -> List[Tuple[str, Optional[dict]]]:
#         """Pop the UI intents accumulated since last call."""
#         out = self._events[:]
#         self._events.clear()
#         return out

#     # ---------- UI LAYOUT ----------
#     def _draw_main_panel(self):
#         flags = imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_ALWAYS_AUTO_RESIZE
#         imgui.set_next_window_position(10, 10, imgui.ONCE)
#         imgui.begin("Control Panel", flags=flags)

#         # --- Header / status ---
#         imgui.text_colored("Industrial Robotics — Pizza Cell", 0.9, 0.9, 0.9)
#         imgui.separator()
#         # Light curtain + pause indicators
#         if self.light_curtain_broken:
#             imgui.text_colored("LIGHT CURTAIN TRIPPED", 1.0, 0.3, 0.3)
#         if self._estop_latched:
#             imgui.text_colored("E-STOP LATCHED — requires Reset then Resume", 1.0, 0.3, 0.3)
#         elif self.system_paused:
#             imgui.text_colored("PAUSED", 1.0, 0.8, 0.0)

#         # --- E-STOP block ---
#         imgui.separator()
#         imgui.text("Emergency Controls")
#         col_push = imgui.push_style_color(imgui.COLOR_BUTTON, 0.8, 0.15, 0.15, 1.0)
#         col_push_h = imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.9, 0.2, 0.2, 1.0)
#         col_push_a = imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.7, 0.1, 0.1, 1.0)
#         if imgui.button("E-STOP", width=200, height=48):
#             # latch and emit event
#             self._estop_latched = True
#             self._events.append(('estop', {'source': 'gui'}))
#         imgui.pop_style_color(3)

#         imgui.same_line()
#         # RESET clears the latch; RESUME is a separate action
#         if imgui.button("Reset", width=90, height=48):
#             if self._estop_latched:
#                 self._estop_latched = False
#                 self._events.append(('estop_reset', None))
#         imgui.same_line()
#         if imgui.button("Resume", width=90, height=48):
#             # only emit resume if *not* latched
#             if not self._estop_latched:
#                 self._events.append(('resume', None))

#         # --- Robot selection ---
#         imgui.separator()
#         imgui.text("Manual Teach (one robot at a time)")
#         if self._robots:
#             current = self._active_robot or list(self._robots.keys())[0]
#             labels = [f"{rid}: {meta['name']}" for rid, meta in self._robots.items()]
#             keys = list(self._robots.keys())
#             sel_idx = keys.index(current)
#             changed, sel_idx = imgui.combo("Active Robot", sel_idx, labels)
#             if changed:
#                 self._active_robot = keys[sel_idx]
#                 self._events.append(('select_robot', {'robot_id': self._active_robot}))
#                 # refresh shadow from robot
#                 try:
#                     self._q_shadow[self._active_robot] = list(self._robots[self._active_robot]['get_q']())
#                 except Exception:
#                     pass

#             # Joint sliders for selected robot
#             rid = self._active_robot
#             if rid is not None:
#                 meta = self._robots[rid]
#                 q = self._q_shadow.get(rid) or list(meta['get_q']())
#                 qlim = meta.get('qlim', [( -math.pi, math.pi)] * len(q))
#                 imgui.text(f"Joint Jog (radians). DOF={len(q)}")
#                 imgui.push_item_width(360)
#                 any_changed = False
#                 for i, val in enumerate(q):
#                     jmin, jmax = qlim[i]
#                     changed, new_val = imgui.slider_float(f"q{i+1}", val, jmin, jmax, format="%.3f")
#                     if changed:
#                         q[i] = new_val
#                         any_changed = True
#                 imgui.pop_item_width()
#                 self._q_shadow[rid] = q

#                 if imgui.button("Apply Joints", width=160):
#                     # Emit set_q — your runner can either apply directly or plan a trajectory
#                     self._events.append(('set_q', {'robot_id': rid, 'q': list(q)}))

#                 imgui.same_line()
#                 if imgui.button("Refresh", width=120):
#                     try:
#                         self._q_shadow[rid] = list(meta['get_q']())
#                     except Exception:
#                         pass

#             # Cartesian nudges for active robot
#             imgui.separator()
#             imgui.text("Cartesian Nudge (EE frame / world frame as you implement)")
#             changed1, self._nudge_lin = imgui.input_float("Δpos (m)", self._nudge_lin, step=0.001, step_fast=0.01)
#             changed2, self._nudge_ang = imgui.input_float("Δrot (rad)", self._nudge_ang, step=math.radians(1), step_fast=math.radians(5))
#             imgui.text("Translate")
#             if imgui.button("+X"): self._enqueue_jog(dx=+self._nudge_lin)
#             imgui.same_line()
#             if imgui.button("-X"): self._enqueue_jog(dx=-self._nudge_lin)
#             imgui.same_line()
#             if imgui.button("+Y"): self._enqueue_jog(dy=+self._nudge_lin)
#             imgui.same_line()
#             if imgui.button("-Y"): self._enqueue_jog(dy=-self._nudge_lin)
#             imgui.same_line()
#             if imgui.button("+Z"): self._enqueue_jog(dz=+self._nudge_lin)
#             imgui.same_line()
#             if imgui.button("-Z"): self._enqueue_jog(dz=-self._nudge_lin)

#             imgui.text("Rotate")
#             if imgui.button("+Roll"):  self._enqueue_jog(droll=+self._nudge_ang)
#             imgui.same_line()
#             if imgui.button("-Roll"):  self._enqueue_jog(droll=-self._nudge_ang)
#             imgui.same_line()
#             if imgui.button("+Pitch"): self._enqueue_jog(dpitch=+self._nudge_ang)
#             imgui.same_line()
#             if imgui.button("-Pitch"): self._enqueue_jog(dpitch=-self._nudge_ang)
#             imgui.same_line()
#             if imgui.button("+Yaw"):   self._enqueue_jog(dyaw=+self._nudge_ang)
#             imgui.same_line()
#             if imgui.button("-Yaw"):   self._enqueue_jog(dyaw=-self._nudge_ang)

#         else:
#             imgui.text_colored("No robots bound. Call bind_robots(...) first.", 1.0, 0.6, 0.4)

#         # --- Safety / status ---
#         imgui.separator()
#         imgui.text("Safety & Status")
#         imgui.checkbox("Light curtain broken (sim)", self._as_checkbox_ref('light_curtain_broken'))
#         imgui.same_line()
#         imgui.text_disabled("(you can wire this to your States each frame)")

#         # --- Quit ---
#         imgui.separator()
#         if imgui.button("Quit Program", width=120):
#             self._events.append(('stop_program', None))

#         imgui.end()

#     # ---------- HELPERS ----------
#     def _enqueue_jog(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0):
#         rid = self._active_robot
#         if rid is None:
#             return
#         self._events.append(
#             ('jog_cart',
#              {'robot_id': rid, 'dx': dx, 'dy': dy, 'dz': dz,
#               'droll': droll, 'dpitch': dpitch, 'dyaw': dyaw})
#         )

#     def _as_checkbox_ref(self, attr_name: str):
#         """
#         Wrap a boolean attr to an ImGui checkbox-style pair (bool ref).
#         Returns (changed, new_value) behavior via a tiny proxy object.
#         """
#         val = getattr(self, attr_name)
#         changed, new_val = imgui.checkbox("", val)
#         if changed:
#             setattr(self, attr_name, new_val)
#         return changed, new_val

#     # (Optional) custom font loader if you have a ttf you like
#     # def _load_font(self, path: str, size_px: int):
#     #     io = imgui.get_io()
#     #     io.fonts.clear()
#     #     io.fonts.add_font_from_file_ttf(path, size_px)
#     #     self._impl.refresh_font_texture()

import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer

def thing():
    # init GLFW
    if not glfw.init():
        return
    window = glfw.create_window(800, 600, "Minimal ImGui", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)
    imgui.create_context()
    impl = GlfwRenderer(window)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        gl.glClearColor(0.1, 0.1, 0.1, 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.new_frame()
        imgui.begin("Hello")
        imgui.text("This is dear ImGui running in Python.")
        if imgui.button("Click me"):
            print("Button pressed!")
        imgui.end()
        imgui.render()

        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    thing()