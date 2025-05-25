#!/usr/bin/env python3
"""
BalanceRoverEnv with softened PID + smoothed torque + on-screen buttons + ω display.
Run with:
    python balance_rover_env.py

Hold ←/→ or click-and-hold the Forward/Backward buttons;
when released, PID takes over to decelerate and re-balance.
"""

import sys
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw
import tkinter as tk

# —————————————————————————————————————————————————————————
# TUNING PARAMETERS
# — PID gains (inner angle loop, gentler than before)
Kp, Ki, Kd =  80.0, 5.0, 2.0

# Manual drive: torque per key/button
DRIVE_GAIN      = 0.5    # N·m (was 1.0 before)
DRIVE_RAMP_RATE = 0.5    # N·m per second (was 1.0)

# Smoothing on total torque: time constant in seconds
SMOOTH_TAU = 0.05        # higher → slower response

MAX_STEPS  = 200_000
FALL_THRESH = np.pi * 0.9
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

        # manual-drive targets & smoothing state
        self.drive_target  = 0.0
        self.drive_current = 0.0

        # final filtered torque command
        self.u_smoothed = 0.0

        # launch MuJoCo GUI
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self._key_cb
        )

        # on-screen buttons + ω display
        self._init_button_ui()

    def _init_button_ui(self):
        self.root = tk.Tk()
        self.root.title("Rover Drive")
        self.root.resizable(False, False)

        btn_fwd = tk.Button(self.root, text="Forward",  width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=0, column=0, padx=5, pady=5)
        btn_bwd.grid(row=0, column=1, padx=5, pady=5)

        self.vel_label = tk.Label(self.root, text="ω = 0.000 rad/s", width=20)
        self.vel_label.grid(row=1, column=0, columnspan=2, pady=(0,5))

        # bind press/release to adjust drive_target
        btn_fwd.bind("<ButtonPress-1>",
                     lambda e: setattr(self, 'drive_target',  self.drive_target + DRIVE_GAIN))
        btn_fwd.bind("<ButtonRelease-1>",
                     lambda e: setattr(self, 'drive_target',  self.drive_target - DRIVE_GAIN))
        btn_bwd.bind("<ButtonPress-1>",
                     lambda e: setattr(self, 'drive_target',  self.drive_target - DRIVE_GAIN))
        btn_bwd.bind("<ButtonRelease-1>",
                     lambda e: setattr(self, 'drive_target',  self.drive_target + DRIVE_GAIN))

    def _key_cb(self, window, key, scancode, action, mods):
        if key == glfw.KEY_RIGHT:
            if action == glfw.PRESS:   self.drive_target += DRIVE_GAIN
            if action == glfw.RELEASE: self.drive_target -= DRIVE_GAIN
        if key == glfw.KEY_LEFT:
            if action == glfw.PRESS:   self.drive_target -= DRIVE_GAIN
            if action == glfw.RELEASE: self.drive_target += DRIVE_GAIN

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[3] = 0.02
        return np.concatenate([self.data.qpos, self.data.qvel])

    def close(self):
        self.viewer.close()
        self.root.destroy()


if __name__ == "__main__":
    env = BalanceRoverEnv()
    dt  = env.model.opt.timestep

    integral, prev_error = 0.0, 0.0

    for step in range(MAX_STEPS):
        # advance physics
        mujoco.mj_step(env.model, env.data)

        # check stability
        if not np.all(np.isfinite(env.data.qacc)):
            print(f"Unstable sim at t={step*dt:.4f}s; aborting.")
            break

        # read pendulum angle & ω
        theta  = env.data.qpos[3]
        theta_dot = env.data.qvel[3]

        # update on-screen ω
        env.vel_label.config(text=f"ω = {theta_dot:.3f} rad/s")

        # — smooth manual-drive input toward its target —
        err_drive = env.drive_target - env.drive_current
        max_delta = DRIVE_RAMP_RATE * dt
        if abs(err_drive) > max_delta:
            env.drive_current += np.sign(err_drive) * max_delta
        else:
            env.drive_current = env.drive_target

        # — inner PID on angle → torque command —
        err       = theta
        integral += err * dt
        derivative = (err - prev_error) / dt
        prev_error = err
        u_pid     = -(Kp*err + Ki*integral + Kd*derivative)

        # — combine PID + drive, then exponential-smooth total —
        u_target = u_pid + env.drive_current
        alpha = dt / SMOOTH_TAU
        env.u_smoothed += alpha * (u_target - env.u_smoothed)

        # clip & send to wheels
        u = float(np.clip(env.u_smoothed, -3.0, 3.0))
        cmd = u / 3.0
        env.data.ctrl[0] = cmd
        env.data.ctrl[1] = cmd

        # render + handle UI events
        env.viewer.sync()
        env.root.update()

        # stop if it falls over
        if abs(theta) > FALL_THRESH:
            print(f"Fallen at t={step*dt:.3f}s (θ={theta:.3f} rad)")
            break

    input("Simulation ended. Press Enter to close…")
    env.close()
