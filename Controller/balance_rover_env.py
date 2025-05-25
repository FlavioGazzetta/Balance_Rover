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
    Kp_vel, Ki_vel, Kd_vel,
    DRIVE_GAIN, DRIVE_RAMP_RATE,
    SMOOTH_TAU, FALL_THRESH,
    SIM_DURATION
)

# Default initial pole tilt (deg)
INITIAL_ANGLE = 10.0
# Dead-band angle around vertical to ignore (deg)
DEAD_BAND_ANGLE = 1.0
# Manual drive decay and overshoot rate (N·m per second)
OVERSHOOT_RATE = 0.05


class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        # Load model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)

        # Find hinge joint qpos index
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
        self.hinge_qposadr = self.model.jnt_qposadr[jid]

        # PID controller with imported defaults
        self.pid = DualPID(Kp_ang, Ki_ang, Kd_ang)

        # State variables
        self.drive_current = 0.0  # holds impulse torque
        self.u_smoothed = 0.0

        # Launch viewer
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self._key_cb
        )

        # Build UI
        self._init_button_ui()

        # Initial respawn
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

        # Parameter specs
        param_specs = [
            ("Kp", 'Kp', 'pid'),
            ("Ki", 'Ki', 'pid'),
            ("Kd", 'Kd', 'pid'),
            ("Kp_vel", 'Kp_vel', 'global'),
            ("Ki_vel", 'Ki_vel', 'global'),
            ("Kd_vel", 'Kd_vel', 'global'),
            ("Drive Gain", 'DRIVE_GAIN', 'global'),
            ("Smooth Tau", 'SMOOTH_TAU', 'global'),
            ("Fall Threshold", 'FALL_THRESH', 'global'),
            ("Sim Duration", 'SIM_DURATION', 'global'),
            ("Initial Angle (deg)", 'INITIAL_ANGLE', 'global'),
            ("Overshoot Rate", 'OVERSHOOT_RATE', 'global')
        ]
        self.entries = {}
        for name, attr, scope in param_specs:
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=8)
            val = getattr(self.pid, attr) if scope=='pid' else globals()[attr]
            entry.insert(0, str(val))
            entry.grid(row=row, column=1, sticky="w")
            self.entries[(scope, attr)] = entry

        # Update & Respawn buttons
        row += 1
        tk.Button(self.root, text="Update Params", command=self._update_params).grid(
            row=row, column=0, columnspan=2, pady=(5,5)
        )
        row += 1
        tk.Button(self.root, text="Respawn Rover", command=self.respawn).grid(
            row=row, column=0, columnspan=2, pady=(5,10)
        )

        # Bind drive buttons
        btn_fwd.bind("<ButtonPress-1>", self._on_fwd_press)
        btn_bwd.bind("<ButtonPress-1>", self._on_bwd_press)

    def _update_params(self):
        try:
            for (scope, attr), entry in self.entries.items():
                val = float(entry.get())
                if scope=='pid': setattr(self.pid, attr, val)
                else: globals()[attr]=val
            print("Parameters updated.")
        except ValueError:
            print("Invalid input; please enter numeric values.")

    def respawn(self):
        # Reset simulation & tilt
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qpos[self.hinge_qposadr] = np.deg2rad(INITIAL_ANGLE)
        mujoco.mj_forward(self.model, self.data)

        # Reset controller & drive
        self.pid.reset()
        self.drive_current = 0.0
        self.u_smoothed = 0.0

    def _on_fwd_press(self, event):
        # One-time forward torque impulse
        self.drive_current += DRIVE_GAIN
    def _on_bwd_press(self, event):
        # One-time backward torque impulse
        self.drive_current -= DRIVE_GAIN

    def _key_cb(self, window, key, scancode, action, mods):
        # no keyboard drive
        pass

    def run(self):
        dt = self.model.opt.timestep
        t = 0.0
        while t < SIM_DURATION:
            mujoco.mj_step(self.model, self.data); t+=dt
            theta = self.data.qpos[self.hinge_qposadr]
            theta_dot = self.data.qvel[self.hinge_qposadr]

                        # PID torque
            u_pid = self.pid.update(theta, theta_dot, dt, self.u_smoothed, (-300000,300000))

            # Apply manual drive decay with overshoot
            if self.drive_current != 0.0:
                dec = OVERSHOOT_RATE * dt
                if abs(self.drive_current) > dec:
                    self.drive_current -= np.sign(self.drive_current) * dec
                else:
                    # overshoot bounce
                    self.drive_current = -np.sign(self.drive_current) * (dec - abs(self.drive_current))

            # Combine & smooth
            u_target = u_pid + self.drive_current
            self.u_smoothed += (dt/SMOOTH_TAU)*(u_target-self.u_smoothed)

            # Apply
            cmd = float(np.clip(self.u_smoothed))
            self.data.ctrl[0]=cmd; self.data.ctrl[1]=cmd

            # UI update
            self.angle_label.config(text=f"Angle = {np.rad2deg(theta):.2f}°")
            self.viewer.sync(); self.root.update()

            if abs(theta)>FALL_THRESH:
                print(f"Fallen at t={t:.3f}s")
        print("Simulation complete.")
        input("Press Enter to close…")
        self.close()

    def close(self):
        self.viewer.close(); self.root.destroy()

if __name__=="__main__":
    BalanceRoverEnv().run()
