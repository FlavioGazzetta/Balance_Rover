#!/usr/bin/env python3
"""
BalanceRoverEnv that delegates control logic to pid_controller.DualPID
with keyboard & button drive + live tuning of all controller parameters + respawn,
starting with the pole at 80° from the ground (10° from vertical), and displaying
the current angle relative to the vertical upward normal.
"""

import sys
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw
import tkinter as tk

# Import the DualPID controller and default gains from the external file
from pid_controller import DualPID, Kp_ang, Ki_ang, Kd_ang

# —————————————————————————————————————————————————————————
# Default controller parameters
DRIVE_GAIN      = 0.2    # N·m per button/key press
DRIVE_RAMP_RATE = 0.2    # N·m per second
SMOOTH_TAU      = 0.5    # seconds for exponential smoothing
FALL_THRESH     = np.pi * 0.9
SIM_DURATION    = 600.0  # seconds of simulation time
# —————————————————————————————————————————————————————————

class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        # load model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)

        # PID controller (angle loop)
        self.pid = DualPID(Kp_ang, Ki_ang, Kd_ang)

        # manual-drive pressed flags
        self.forward_pressed  = False
        self.backward_pressed = False

        # drive targets & smoothing state
        self.drive_target  = 0.0
        self.drive_current = 0.0
        self.u_smoothed    = 0.0

        # launch MuJoCo window
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self._key_cb
        )

        # build control UI
        self._init_button_ui()

        # initial perfect respawn
        self.respawn()

    def _init_button_ui(self):
        """Create Tk window with drive buttons, parameter entries, angle display, update & respawn."""
        self.root = tk.Tk()
        self.root.title("Rover Drive + PID & Params Tuning")
        self.root.resizable(False, False)

        row = 0
        # Drive buttons
        btn_fwd = tk.Button(self.root, text="Forward",  width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=row, column=0, padx=5, pady=5)
        btn_bwd.grid(row=row, column=1, padx=5, pady=5)

        # Angle display (relative to vertical)
        row += 1
        self.angle_label = tk.Label(self.root, text="Angle = 0.00°", width=20)
        self.angle_label.grid(row=row, column=0, columnspan=2, pady=(0,10))

        # PID entries
        for name, attr in [("Kp", "pid.Kp"), ("Ki", "pid.Ki"), ("Kd", "pid.Kd")]:
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=6)
            entry.insert(0, str(getattr(self.pid, name)))
            entry.grid(row=row, column=1, sticky="w")
            setattr(self, f"entry_{name}", entry)

        # Other parameter entries
        for label, var in [("Drive Gain", "DRIVE_GAIN"), ("Drive Ramp", "DRIVE_RAMP_RATE"),
                           ("Smooth Tau", "SMOOTH_TAU"), ("Fall Threshold (rad)", "FALL_THRESH"),
                           ("Sim Duration (s)", "SIM_DURATION")]:
            row += 1
            tk.Label(self.root, text=f"{label}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=8)
            entry.insert(0, str(globals()[var]))
            entry.grid(row=row, column=1, sticky="w")
            setattr(self, f"entry_{var}", entry)

        # Update Params button
        row += 1
        btn_update = tk.Button(self.root, text="Update All Params", command=self._update_params)
        btn_update.grid(row=row, column=0, columnspan=2, pady=(5,5))

        # Respawn button
        row += 1
        btn_respawn = tk.Button(self.root, text="Respawn Rover", command=self.respawn)
        btn_respawn.grid(row=row, column=0, columnspan=2, pady=(5,10))

        # Bind drive button events
        btn_fwd.bind("<ButtonPress-1>",   self._on_fwd_press)
        btn_fwd.bind("<ButtonRelease-1>", self._on_fwd_release)
        btn_bwd.bind("<ButtonPress-1>",   self._on_bwd_press)
        btn_bwd.bind("<ButtonRelease-1>", self._on_bwd_release)

    def _update_params(self):
        """Read all entries and apply new gains + parameters."""
        try:
            # PID
            for name in ("Kp", "Ki", "Kd"):  # update PID gains
                val = float(getattr(self, f"entry_{name}").get())
                setattr(self.pid, name, val)
            # Other params
            globals()["DRIVE_GAIN"]      = float(self.entry_DRIVE_GAIN.get())
            globals()["DRIVE_RAMP_RATE"] = float(self.entry_DRIVE_RAMP_RATE.get())
            globals()["SMOOTH_TAU"]      = float(self.entry_SMOOTH_TAU.get())
            globals()["FALL_THRESH"]     = float(self.entry_FALL_THRESH.get())
            globals()["SIM_DURATION"]    = float(self.entry_SIM_DURATION.get())
            print("Parameters updated.")
        except ValueError:
            print("Invalid input! Please enter numeric values.")

    def respawn(self):
        """Reset rover to upright state at 10° from vertical (80° from horizontal)."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qpos[3] = np.deg2rad(0.0)
        mujoco.mj_forward(self.model, self.data)
        self.pid.reset()
        self.drive_target = 0.0
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.forward_pressed = False
        self.backward_pressed = False

    def _on_fwd_press(self, event):
        self.forward_pressed = True
        self._update_drive_target()
    def _on_fwd_release(self, event):
        self.forward_pressed = False
        self._update_drive_target()
    def _on_bwd_press(self, event):
        self.backward_pressed = True
        self._update_drive_target()
    def _on_bwd_release(self, event):
        self.backward_pressed = False
        self._update_drive_target()
    def _update_drive_target(self):
        self.drive_target = DRIVE_GAIN * (1 if self.forward_pressed else 0) \
                          - DRIVE_GAIN * (1 if self.backward_pressed else 0)
    def _key_cb(self, window, key, scancode, action, mods):
        if key == glfw.KEY_RIGHT:
            self.forward_pressed = action==glfw.PRESS
        if key == glfw.KEY_LEFT:
            self.backward_pressed = action==glfw.PRESS
        self._update_drive_target()
    def close(self):
        self.viewer.close()
        self.root.destroy()

if __name__ == "__main__":
    env = BalanceRoverEnv()
    dt = env.model.opt.timestep
    sim_time = 0.0
    while sim_time < SIM_DURATION:
        mujoco.mj_step(env.model, env.data)
        sim_time += dt
        theta = env.data.qpos[3]
        theta_dot = env.data.qvel[3]
        u_pid = -env.pid.update(theta, theta_dot, dt, env.u_smoothed, (-3.0,3.0))
        err_d = env.drive_target - env.drive_current
        mdx = DRIVE_RAMP_RATE * dt
        env.drive_current += np.sign(err_d)*min(abs(err_d), mdx)
        u_target = u_pid + env.drive_current
        env.u_smoothed += (dt/SMOOTH_TAU)*(u_target-env.u_smoothed)
        u = float(np.clip(env.u_smoothed, -3.0, 3.0))
        cmd = u/3.0
        env.data.ctrl[0]=cmd; env.data.ctrl[1]=cmd
        env.angle_label.config(text=f"Angle = {np.rad2deg(theta):.2f}°")
        env.viewer.sync(); env.root.update()
        if abs(theta)>FALL_THRESH:
            print(f"Fallen at t={sim_time:.3f}s (θ={theta:.3f} rad)")
    print("Simulation time reached.")
    input("Press Enter to close…")
    env.close()
