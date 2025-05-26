#=== File: /c/Users/User/Documents/Balance_Rover/Controller/balance_rover_env.py ===
#!/usr/bin/env python3
"""
BalanceRoverEnv that delegates control logic to pid_controller.DualPID
with keyboard & button drive + live tuning of all controller parameters + respawn,
starting with the pole at 80° from the ground (10° from vertical), and displaying
the current angle relative to the vertical upward normal, and continuously applying
a corrective torque proportional to the average slide velocity (in the opposite direction)
so that the rover hovers around the same spot.
"""

import sys
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw
import tkinter as tk
from collections import deque

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
# Manual drive decay and overshoot rate (N·m per second)
OVERSHOOT_RATE = 0.05
# Window (s) to average slide velocity
DRIFT_WINDOW = 5.0
# Proportional gain for drift correction (N·m per (m/s) drift)
DRIFT_GAIN = -0.4   # tune this to your liking


class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        # Load model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)

        # Joint indices
        hinge_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
        slide_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'slide')
        self.hinge_qposadr = self.model.jnt_qposadr[hinge_jid]
        self.slide_qposadr = self.model.jnt_qposadr[slide_jid]

        # PID for pole balance
        self.pid = DualPID(Kp_ang, Ki_ang, Kd_ang)

        # Drive state
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.manual_active = False

        # Drift buffer over last DRIFT_WINDOW seconds
        self.max_buf_len = int(DRIFT_WINDOW / self.model.opt.timestep)
        self.drift_buffer = deque(maxlen=self.max_buf_len)

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

        row = 0
        # Forward/back buttons
        btn_fwd = tk.Button(self.root, text="Forward", width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=row, column=0, padx=5, pady=5)
        btn_bwd.grid(row=row, column=1, padx=5, pady=5)

        # Angle & drift display
        row += 1
        self.angle_label = tk.Label(
            self.root, text="Angle = 0.00°, Drift = 0.0000 m/s", width=40
        )
        self.angle_label.grid(row=row, column=0, columnspan=2, pady=(0,10))

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
        ]
        self.entries = {}
        for name, attr, scope in param_specs:
            row += 1
            tk.Label(self.root, text=f"{name}:").grid(
                row=row, column=0, sticky="e"
            )
            entry = tk.Entry(self.root, width=8)
            val = getattr(self.pid, attr) if scope == 'pid' else globals()[attr]
            entry.insert(0, str(val))
            entry.grid(row=row, column=1, sticky="w")
            self.entries[(scope, attr)] = entry

        # Update & respawn buttons
        row += 1
        tk.Button(
            self.root, text="Update Params", command=self._update_params
        ).grid(row=row, column=0, columnspan=2, pady=(5,5))
        row += 1
        tk.Button(
            self.root, text="Respawn Rover", command=self.respawn
        ).grid(row=row, column=0, columnspan=2, pady=(5,10))

        # Bind button presses/releases
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
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        # initial pole tilt
        self.data.qpos[self.hinge_qposadr] = np.deg2rad(INITIAL_ANGLE)
        mujoco.mj_forward(self.model, self.data)

        # reset drive & PID
        self.pid.reset()
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.manual_active = False
        self.drift_buffer.clear()

    def _on_fwd_press(self, event):
        self.manual_active = True
        self.drive_current += DRIVE_GAIN

    def _on_fwd_release(self, event):
        self.manual_active = False

    def _on_bwd_press(self, event):
        self.manual_active = True
        self.drive_current -= DRIVE_GAIN

    def _on_bwd_release(self, event):
        self.manual_active = False

    def _key_cb(self, window, key, scancode, action, mods):
        pass  # no keyboard drive

    def run(self):
        dt = self.model.opt.timestep
        t = 0.0

        while t < SIM_DURATION:
            mujoco.mj_step(self.model, self.data)
            t += dt

            # sample and buffer slide velocity
            v = float(self.data.qvel[self.slide_qposadr])
            self.drift_buffer.append(v)
            v_avg = sum(self.drift_buffer) / len(self.drift_buffer)

            # apply continuous proportional correction opposite to avg drift
            if not self.manual_active:
                self.drive_current = -v_avg * DRIFT_GAIN

            # manual drive decay / overshoot bounce
            if self.drive_current != 0.0:
                dec = OVERSHOOT_RATE * dt
                if abs(self.drive_current) > dec:
                    self.drive_current -= np.sign(self.drive_current) * dec
                else:
                    self.drive_current = -np.sign(self.drive_current) * (dec - abs(self.drive_current))

            # combine & smooth with PID for pole
            theta = self.data.qpos[self.hinge_qposadr]
            theta_dot = self.data.qvel[self.hinge_qposadr]
            u_pid = self.pid.update(
                theta, theta_dot, dt, self.u_smoothed, (-300000,300000)
            )
            u_target = u_pid + self.drive_current
            self.u_smoothed += (dt/SMOOTH_TAU) * (u_target - self.u_smoothed)

            # apply to wheels
            cmd = float(np.clip(self.u_smoothed))
            self.data.ctrl[0] = cmd
            self.data.ctrl[1] = cmd

            # update UI
            self.angle_label.config(text=(
                f"Angle = {np.rad2deg(theta):.2f}°, Drift = {v_avg:.4f} m/s"
            ))
            self.viewer.sync()
            self.root.update()

            if abs(theta) > FALL_THRESH:
                print(f"Fallen at t={t:.3f}s")

        print("Simulation complete.")
        input("Press Enter to close…")
        self.close()

    def close(self):
        self.viewer.close()
        self.root.destroy()


if __name__ == "__main__":
    BalanceRoverEnv().run()
