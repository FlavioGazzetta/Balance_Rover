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

# Import controller logic and default parameters
from pid_controller import (
    DualPID,
    Kp_ang, Ki_ang, Kd_ang,
    DRIVE_GAIN, DRIVE_RAMP_RATE,
    SMOOTH_TAU, FALL_THRESH,
    SIM_DURATION
)

class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        # load model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)

        # find hinge qpos index
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
        self.hinge_qposadr = self.model.jnt_qposadr[jid]

        # PID controller with imported defaults
        self.pid = DualPID(Kp_ang, Ki_ang, Kd_ang)

        # state variables
        self.forward_pressed = False
        self.backward_pressed = False
        self.drive_target = 0.0
        self.drive_current = 0.0
        self.u_smoothed = 0.0

        # launch viewer
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self._key_cb)

        # build UI
        self._init_button_ui()

        # initial respawn
        self.respawn()

    def _init_button_ui(self):
        self.root = tk.Tk()
        self.root.title("Rover Control & Params")
        self.root.resizable(False, False)

        row = 0
        # Drive buttons
        btn_fwd = tk.Button(self.root, text="Forward", width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=row, column=0, padx=5, pady=5)
        btn_bwd.grid(row=row, column=1, padx=5, pady=5)

        # Angle display
        row += 1
        self.angle_label = tk.Label(self.root, text="Angle = 0.00°", width=20)
        self.angle_label.grid(row=row, column=0, columnspan=2, pady=(0,10))

        # Parameter specifications: (label, attribute, scope)
        param_specs = [
            ("Kp", 'Kp', 'pid'),
            ("Ki", 'Ki', 'pid'),
            ("Kd", 'Kd', 'pid'),
            ("Drive Gain", 'DRIVE_GAIN', 'global'),
            ("Drive Ramp", 'DRIVE_RAMP_RATE', 'global'),
            ("Smooth Tau", 'SMOOTH_TAU', 'global'),
            ("Fall Threshold", 'FALL_THRESH', 'global'),
            ("Sim Duration", 'SIM_DURATION', 'global')
        ]
        self.entries = {}
        for name, attr, scope in param_specs:
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=8)
            if scope == 'pid':
                value = getattr(self.pid, attr)
            else:
                value = globals()[attr]
            entry.insert(0, str(value))
            entry.grid(row=row, column=1, sticky="w")
            self.entries[(scope, attr)] = entry

        # Update Params button
        row += 1
        tk.Button(self.root, text="Update Params", command=self._update_params).grid(
            row=row, column=0, columnspan=2, pady=(5,5)
        )
        # Respawn button
        row += 1
        tk.Button(self.root, text="Respawn Rover", command=self.respawn).grid(
            row=row, column=0, columnspan=2, pady=(5,10)
        )

        # Bind drive button events
        btn_fwd.bind("<ButtonPress-1>", self._on_fwd_press)
        btn_fwd.bind("<ButtonRelease-1>", self._on_fwd_release)
        btn_bwd.bind("<ButtonPress-1>", self._on_bwd_press)
        btn_bwd.bind("<ButtonRelease-1>", self._on_bwd_release)

    def _update_params(self):
        try:
            for (scope, attr), entry in self.entries.items():
                val = float(entry.get())
                if scope == 'pid':
                    setattr(self.pid, attr, val)
                else:
                    globals()[attr] = val
            print("Parameters updated.")
        except ValueError:
            print("Invalid input; please enter numeric values.")

    def respawn(self):
        # Reset simulation state and tilt pole to 10° from vertical
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qpos[self.hinge_qposadr] = np.deg2rad(0.0)
        mujoco.mj_forward(self.model, self.data)

        # Reset controller and drive states
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
        # Manual drive inversion: pressing forward applies negative torque
        self.drive_target = DRIVE_GAIN * (1 if self.backward_pressed else 0) \
                          - DRIVE_GAIN * (1 if self.forward_pressed else 0)

    def _key_cb(self, window, key, scancode, action, mods):
        # Keyboard drive mapping
        if key == glfw.KEY_RIGHT:
            self.forward_pressed = (action == glfw.PRESS)
        if key == glfw.KEY_LEFT:
            self.backward_pressed = (action == glfw.PRESS)
        self._update_drive_target()

    def run(self):
        dt = self.model.opt.timestep
        sim_time = 0.0
        while sim_time < SIM_DURATION:
            mujoco.mj_step(self.model, self.data)
            sim_time += dt

            # Read pole angle and velocity
            theta = self.data.qpos[self.hinge_qposadr]
            theta_dot = self.data.qvel[self.hinge_qposadr]

            # PID control torque (positive => rotate pole back upright)
            u_pid = self.pid.update(theta, theta_dot, dt, self.u_smoothed, (-3.0, 3.0))

            # Manual drive ramping
            err_d = self.drive_target - self.drive_current
            ramp = min(abs(err_d), DRIVE_RAMP_RATE * dt)
            self.drive_current += np.sign(err_d) * ramp

            # Combine and smooth
            u_target = u_pid + self.drive_current
            alpha = dt / SMOOTH_TAU
            self.u_smoothed += alpha * (u_target - self.u_smoothed)

            # Apply controls
            cmd = float(np.clip(self.u_smoothed, -3.0, 3.0)) / 3.0
            self.data.ctrl[0] = cmd
            self.data.ctrl[1] = cmd

            # Update UI
            self.angle_label.config(text=f"Angle = {np.rad2deg(theta):.2f}°")
            self.viewer.sync()
            self.root.update()

            # Log falls
            if abs(theta) > FALL_THRESH:
                print(f"Fallen at t={sim_time:.3f}s (θ={theta:.3f} rad)")

        print("Simulation complete.")
        input("Press Enter to close…")
        self.close()

    def close(self):
        self.viewer.close()
        self.root.destroy()

if __name__ == "__main__":
    env = BalanceRoverEnv()
    env.run()