#!/usr/bin/env python3
import sys, os
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer
from mujoco.viewer import glfw
import tkinter as tk
from collections import deque
from ctypes import CDLL, c_void_p, c_float
import numpy.ctypeslib as ctl

# —————————————————————————————————————————————————————————
# 1) Load & bind C++ controller library (only the functions that exist)
# —————————————————————————————————————————————————————————
script_dir = Path(__file__).parent
dll_path   = script_dir / "controller_lib.dll"
if not dll_path.exists():
    print(f"Error: controller_lib.dll not found at {dll_path}")
    sys.exit(1)

# (Windows DLL search paths)
mujoco_lib = os.getenv('MUJOCO_LIB', r'C:\mujoco\mujoco210\bin')
mingw_bin  = r'C:\msys64\mingw64\bin'
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(str(script_dir))
    if os.path.isdir(mujoco_lib): os.add_dll_directory(mujoco_lib)
    if os.path.isdir(mingw_bin):  os.add_dll_directory(mingw_bin)
else:
    os.environ['PATH'] = os.pathsep.join([
        str(script_dir), mujoco_lib, mingw_bin,
        os.environ.get('PATH','')
    ])

lib = CDLL(str(dll_path))

# Factory / destroyer
lib.make_controller.restype = c_void_p
lib.free_controller.argtypes = [c_void_p]

# Step: theta, theta_dot, slide_vel, dt → cmd
float_ptr = ctl.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
lib.update_controller.argtypes = [c_void_p, float_ptr, float_ptr, float_ptr, float_ptr]
lib.update_controller.restype  = c_float

# Manual‐drive callbacks
for fn in ("on_fwd_press","on_fwd_release","on_bwd_press","on_bwd_release"):
    f = getattr(lib, fn)
    f.argtypes = [c_void_p]
    f.restype  = None

# —————————————————————————————————————————————————————————
# 2) Python-side constants (used for UI defaults & simulation limits)
#    (These must match your C++ defaults!)
# —————————————————————————————————————————————————————————
from math import pi
INITIAL_ANGLE   = 10.0 * pi/180.0
FALL_THRESH     = pi * 0.9
SIM_DURATION    = 600.0
DRIFT_WINDOW    = 5.0

# We show these in the UI but *cannot* actually reconfigure
# them at runtime in C++ without additional setters.
PARAM_DEFAULTS = {
    "Kp_ang":       30.0,
    "Ki_ang":       10.0,
    "Kd_ang":       0.5,
    "DRIVE_GAIN":      0.4,
    "DRIVE_RAMP_RATE": 0.2,
    "DRIFT_WINDOW":    DRIFT_WINDOW,
    "DRIFT_GAIN":      -0.4,
    "SMOOTH_TAU":      4.0,
    "FALL_THRESH":     FALL_THRESH,
    "SIM_DURATION":    SIM_DURATION,
    "OVERSHOOT_RATE":  0.05,
    "INITIAL_ANGLE":   INITIAL_ANGLE * (180.0/pi),  # show in degrees
}

# —————————————————————————————————————————————————————————
# 3) BalanceRoverEnv exactly as before, but backed by C++
# —————————————————————————————————————————————————————————
class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        # Load model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)
        dt = self.model.opt.timestep

        # C++ controller instance
        self.ctrl_ptr = lib.make_controller()

        # Joint indices
        hinge_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
        slide_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'slide')
        self.hinge_qposadr = self.model.jnt_qposadr[hinge_jid]
        self.slide_qposadr = self.model.jnt_qposadr[slide_jid]

        # Viewer + UI
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self._key_cb
        )
        self._init_button_ui()

        # Drift buffer
        max_len = int(DRIFT_WINDOW / dt)
        self.drift_buffer = deque(maxlen=max_len)

        # Initial respawn
        self.respawn()

    def _init_button_ui(self):
        self.root = tk.Tk()
        self.root.title("Rover Control & Params")
        self.root.resizable(False, False)

        # Forward/back buttons
        btn_fwd = tk.Button(self.root, text="Forward",  width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=0, column=0, padx=5, pady=5)
        btn_bwd.grid(row=0, column=1, padx=5, pady=5)

        # Angle & drift display
        self.angle_label = tk.Label(
            self.root,
            text="Angle = 0.00°, Drift = 0.0000 m/s",
            width=40
        )
        self.angle_label.grid(row=1, column=0, columnspan=2, pady=(0,10))

        # Parameter entries (display only)
        self.entries = {}
        row = 1
        for name, val in PARAM_DEFAULTS.items():
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=10)
            entry.insert(0, f"{val:.4g}")
            entry.config(state='readonly')
            entry.grid(row=row, column=1, sticky="w")
            self.entries[name] = entry

        # Update & respawn buttons
        row += 1
        tk.Button(
            self.root,
            text="Update Params",
            command=lambda: print("Live tuning not supported without C++ setters.")
        ).grid(row=row, column=0, columnspan=2, pady=(5,5))
        row += 1
        tk.Button(
            self.root,
            text="Respawn Rover",
            command=self.respawn
        ).grid(row=row, column=0, columnspan=2, pady=(5,10))

        # Bind press/release to C++ handlers
        btn_fwd.bind("<ButtonPress-1>",  lambda e: lib.on_fwd_press(self.ctrl_ptr))
        btn_fwd.bind("<ButtonRelease-1>",lambda e: lib.on_fwd_release(self.ctrl_ptr))
        btn_bwd.bind("<ButtonPress-1>",  lambda e: lib.on_bwd_press(self.ctrl_ptr))
        btn_bwd.bind("<ButtonRelease-1>",lambda e: lib.on_bwd_release(self.ctrl_ptr))

    def _key_cb(self, window, key, scancode, action, mods):
        # disable keyboard drive
        pass

    def respawn(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0

        # initial pole tilt
        self.data.qpos[self.hinge_qposadr] = INITIAL_ANGLE
        mujoco.mj_forward(self.model, self.data)

        # reset C++ controller
        lib.free_controller(self.ctrl_ptr)
        self.ctrl_ptr = lib.make_controller()
        self.drift_buffer.clear()

    def run(self):
        dt = self.model.opt.timestep
        t  = 0.0

        while t < SIM_DURATION:
            mujoco.mj_step(self.model, self.data)
            t += dt

            theta     = np.array([self.data.qpos[self.hinge_qposadr]], dtype=np.float32)
            theta_dot = np.array([self.data.qvel[self.hinge_qposadr]], dtype=np.float32)
            slide_vel = np.array([self.data.qvel[self.slide_qposadr]], dtype=np.float32)
            dt_arr    = np.array([dt], dtype=np.float32)

            # C++ step → cmd
            cmd = float(lib.update_controller(
                self.ctrl_ptr, theta, theta_dot, slide_vel, dt_arr
            ))

            # update drift display
            self.drift_buffer.append(slide_vel[0])
            drift = float(sum(self.drift_buffer)/len(self.drift_buffer))

            self.data.ctrl[0] = cmd
            self.data.ctrl[1] = cmd

            self.angle_label.config(
                text=f"Angle = {np.rad2deg(theta[0]):.2f}°, Drift = {drift:.4f} m/s"
            )
            self.viewer.sync()
            self.root.update()

            if abs(theta[0]) > FALL_THRESH:
                print(f"Fallen at t={t:.3f}s")
                break

        print("Simulation complete.")
        input("Press Enter to close…")
        self.close()

    def close(self):
        lib.free_controller(self.ctrl_ptr)
        self.viewer.close()
        self.root.destroy()


if __name__ == "__main__":
    BalanceRoverEnv().run()
