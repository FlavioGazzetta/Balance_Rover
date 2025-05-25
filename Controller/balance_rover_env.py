#!/usr/bin/env python3
"""
BalanceRoverEnv with PID + keyboard & button drive + live PID tuning + respawn,
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

# —————————————————————————————————————————————————————————
# INITIAL TUNING PARAMETERS (live-tunable via UI)
Kp, Ki, Kd = 30.0, 1.0, 0.5
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

        # PID integrator state
        self.integral   = 0.0
        self.prev_error = 0.0

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
        """Create Tk window with drive buttons, angle display, PID inputs, and respawn."""
        self.root = tk.Tk()
        self.root.title("Rover Drive + PID Tuning")
        self.root.resizable(False, False)

        # Drive buttons
        btn_fwd = tk.Button(self.root, text="Forward",  width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=0, column=0, padx=5, pady=5)
        btn_bwd.grid(row=0, column=1, padx=5, pady=5)

        # Angle display (relative to vertical)
        self.angle_label = tk.Label(self.root, text="Angle = 0.00°", width=20)
        self.angle_label.grid(row=1, column=0, columnspan=2, pady=(0,10))

        # PID entries
        tk.Label(self.root, text="Kp:").grid(row=2, column=0, sticky="e")
        self.entry_Kp = tk.Entry(self.root, width=6)
        self.entry_Kp.insert(0, str(Kp))
        self.entry_Kp.grid(row=2, column=1, sticky="w")

        tk.Label(self.root, text="Ki:").grid(row=3, column=0, sticky="e")
        self.entry_Ki = tk.Entry(self.root, width=6)
        self.entry_Ki.insert(0, str(Ki))
        self.entry_Ki.grid(row=3, column=1, sticky="w")

        tk.Label(self.root, text="Kd:").grid(row=4, column=0, sticky="e")
        self.entry_Kd = tk.Entry(self.root, width=6)
        self.entry_Kd.insert(0, str(Kd))
        self.entry_Kd.grid(row=4, column=1, sticky="w")

        # Update PID button
        btn_update = tk.Button(self.root, text="Update PID Gains", command=self._update_pid)
        btn_update.grid(row=5, column=0, columnspan=2, pady=(5,5))

        # Respawn button
        btn_respawn = tk.Button(self.root, text="Respawn Rover", command=self.respawn)
        btn_respawn.grid(row=6, column=0, columnspan=2, pady=(5,10))

        # Bind drive button events
        btn_fwd.bind("<ButtonPress-1>",   self._on_fwd_press)
        btn_fwd.bind("<ButtonRelease-1>", self._on_fwd_release)
        btn_bwd.bind("<ButtonPress-1>",   self._on_bwd_press)
        btn_bwd.bind("<ButtonRelease-1>", self._on_bwd_release)

    def _update_pid(self):
        """Read PID entries and apply new gains."""
        global Kp, Ki, Kd
        try:
            Kp = float(self.entry_Kp.get())
            Ki = float(self.entry_Ki.get())
            Kd = float(self.entry_Kd.get())
            print(f"PID gains updated: Kp={Kp}, Ki={Ki}, Kd={Kd}")
        except ValueError:
            print("Invalid PID input! Please enter numeric values.")

    def respawn(self):
        """Reset rover to upright state at 10° from vertical (80° from horizontal)."""
        mujoco.mj_resetData(self.model, self.data)

        # Zero all qpos, qvel, controls
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0

        # Set initial pole angle: 10° from vertical => 80° from ground
        initial_angle_deg = 10.0
        initial_angle_rad = np.deg2rad(initial_angle_deg)
        # qpos[3] corresponds to the pole hinge joint
        self.data.qpos[3] = initial_angle_rad

        mujoco.mj_forward(self.model, self.data)

        # Reset controller states
        self.integral         = 0.0
        self.prev_error       = 0.0
        self.drive_target     = 0.0
        self.drive_current    = 0.0
        self.u_smoothed       = 0.0
        self.forward_pressed  = False
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
        """Handle ←/→ key presses for drive."""
        if key == glfw.KEY_RIGHT:
            if action == glfw.PRESS:
                self.forward_pressed = True
                self._update_drive_target()
            elif action == glfw.RELEASE:
                self.forward_pressed = False
                self._update_drive_target()
        if key == glfw.KEY_LEFT:
            if action == glfw.PRESS:
                self.backward_pressed = True
                self._update_drive_target()
            elif action == glfw.RELEASE:
                self.backward_pressed = False
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

        if not np.all(np.isfinite(env.data.qacc)):
            print(f"Numerical instability at t={sim_time:.3f}s; aborting early.")
            break

        # Read pendulum angle & angular velocity
        theta       = env.data.qpos[3]      # radians from vertical
        theta_dot   = env.data.qvel[3]
        theta_deg   = np.rad2deg(theta)
        # Update display: angle relative to vertical upward normal
        self_text   = f"Angle = {theta_deg:.2f}°"
        env.angle_label.config(text=self_text)

        # Ramp manual drive
        err_d = env.drive_target - env.drive_current
        mdx   = DRIVE_RAMP_RATE * dt
        if abs(err_d) > mdx:
            env.drive_current += np.sign(err_d) * mdx
        else:
            env.drive_current = env.drive_target

        # PID on angle → torque
        err             = theta
        env.integral   += err * dt
        derivative      = (err - env.prev_error) / dt
        env.prev_error  = err
        u_pid           = -(Kp * err + Ki * env.integral + Kd * derivative)

        # Combine & exponential-smooth
        u_target        = u_pid + env.drive_current
        alpha           = dt / SMOOTH_TAU
        env.u_smoothed += alpha * (u_target - env.u_smoothed)

        # Apply to wheels
        u   = float(np.clip(env.u_smoothed, -3.0, 3.0))
        cmd = u / 3.0
        env.data.ctrl[0] = cmd
        env.data.ctrl[1] = cmd

        # Render & UI
        env.viewer.sync()
        env.root.update()

        # If it falls, note but keep running
        if abs(theta) > FALL_THRESH:
            print(f"Fallen at t={sim_time:.3f}s (θ={theta:.3f} rad)")

    print("Simulation time reached.")
    input("Press Enter to close…")
    env.close()
