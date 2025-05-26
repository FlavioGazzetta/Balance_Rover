#!/usr/bin/env python3
"""
BalanceRoverEnv: environment setup, UI, and simulation loop.
Delegates all control logic to controller.RoverController.
Allows live tuning of all controller parameters via UI.
"""

import sys
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw
import tkinter as tk

import pid_controller as controller
from pid_controller import RoverController, INITIAL_ANGLE, SIM_DURATION, FALL_THRESH

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

        # Controller
        self.ctrl = RoverController(dt)

        # Joint indices
        hinge_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
        slide_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'slide')
        self.hinge_qposadr = self.model.jnt_qposadr[hinge_jid]
        self.slide_qposadr = self.model.jnt_qposadr[slide_jid]

        # Launch viewer & UI
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self._key_cb
        )
        self._init_button_ui()

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
        self.angle_label = tk.Label(self.root, text="Angle = 0.00°, Drift = 0.0000 m/s", width=40)
        self.angle_label.grid(row=1, column=0, columnspan=2, pady=(0,10))

        # Parameter entries
        param_specs = [
            ("Kp", 'Kp', 'pid'),
            ("Ki", 'Ki', 'pid'),
            ("Kd", 'Kd', 'pid'),
            ("Drive Gain", 'DRIVE_GAIN', 'global'),
            ("Drive Ramp Rate", 'DRIVE_RAMP_RATE', 'global'),
            ("Drift Window", 'DRIFT_WINDOW', 'global'),
            ("Drift Gain", 'DRIFT_GAIN', 'global'),
            ("Smooth Tau", 'SMOOTH_TAU', 'global'),
            ("Fall Threshold", 'FALL_THRESH', 'global'),
            ("Sim Duration", 'SIM_DURATION', 'global'),
            ("Overshoot Rate", 'OVERSHOOT_RATE', 'global'),
            ("Initial Angle", 'INITIAL_ANGLE', 'global'),
        ]
        self.entries = {}
        row = 2
        for name, attr, scope in param_specs:
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(row=row, column=0, sticky="e")
            entry = tk.Entry(self.root, width=8)
            if scope == 'pid':
                val = getattr(self.ctrl.pid, attr)
            else:
                val = getattr(controller, attr)
            entry.insert(0, str(val))
            entry.grid(row=row, column=1, sticky="w")
            self.entries[(scope, attr)] = entry

        # Update & respawn
        row += 1
        tk.Button(self.root, text="Update Params", command=self._update_params).grid(row=row, column=0, columnspan=2, pady=(5,5))
        row += 1
        tk.Button(self.root, text="Respawn Rover", command=self.respawn).grid(row=row, column=0, columnspan=2, pady=(5,10))

        # Bind buttons
        btn_fwd.bind("<ButtonPress-1>",  lambda e: self.ctrl.on_fwd_press())
        btn_fwd.bind("<ButtonRelease-1>",lambda e: self.ctrl.on_fwd_release())
        btn_bwd.bind("<ButtonPress-1>",  lambda e: self.ctrl.on_bwd_press())
        btn_bwd.bind("<ButtonRelease-1>",lambda e: self.ctrl.on_bwd_release())

    def _update_params(self):
        try:
            for (scope, attr), entry in self.entries.items():
                val = float(entry.get())
                if scope == 'pid':
                    setattr(self.ctrl.pid, attr, val)
                else:
                    setattr(controller, attr, val)
            print("Parameters updated.")
        except ValueError:
            print("Invalid input; please enter numeric values.")

    def _key_cb(self, window, key, scancode, action, mods):
        # No keyboard drive
        pass

    def respawn(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        # initial pole tilt
        self.data.qpos[self.hinge_qposadr] = np.deg2rad(controller.INITIAL_ANGLE)
        mujoco.mj_forward(self.model, self.data)

        # Reset controller state
        self.ctrl.reset()

    def run(self):
        dt = self.model.opt.timestep
        t = 0.0

        while t < controller.SIM_DURATION:
            mujoco.mj_step(self.model, self.data)
            t += dt

            theta     = self.data.qpos[self.hinge_qposadr]
            theta_dot = self.data.qvel[self.hinge_qposadr]
            slide_vel = float(self.data.qvel[self.slide_qposadr])

            cmd, drift = self.ctrl.step(theta, theta_dot, slide_vel, dt)

            self.data.ctrl[0] = cmd
            self.data.ctrl[1] = cmd

            self.angle_label.config(text=(
                f"Angle = {np.rad2deg(theta):.2f}°, Drift = {drift:.4f} m/s"
            ))
            self.viewer.sync()
            self.root.update()

            if abs(theta) > controller.FALL_THRESH:
                print(f"Fallen at t={t:.3f}s")

        print("Simulation complete.")
        input("Press Enter to close…")
        self.close()

    def close(self):
        self.viewer.close()
        self.root.destroy()


if __name__ == "__main__":
    BalanceRoverEnv().run()
