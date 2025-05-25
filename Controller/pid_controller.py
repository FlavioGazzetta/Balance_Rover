#!/usr/bin/env python3
"""
BalanceRoverEnv with dual-loop PID + advanced control features:
1. Anti-Windup
2. Derivative Filtering
3. Reduced Dead-Band to allow small overshoot
4. Bias Term to encourage slight overshoot
5. Dead-Band/Hysteresis
7. Cascade Structure (angle→velocity→torque)
10. Soft-Start & Ramp-Limits
11. Simple Disturbance Observer
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
# Outer loop (angle) gains
Kp_ang, Ki_ang, Kd_ang = 30.0, 1.0, 0.5
# Inner loop (velocity) gains
Kp_vel, Ki_vel, Kd_vel = 20.0, 0.5, 0.1

DRIVE_GAIN      = 0.2    # N·m per button/key press
DRIVE_RAMP_RATE = 0.2    # N·m per second
SMOOTH_TAU      = 0.5    # seconds for exp smoothing
FALL_THRESH     = np.pi * 0.9
SIM_DURATION    = 600.0  # seconds

# Dead-band threshold (rad) — reduced to allow small overshoot
DEAD_BAND = np.deg2rad(0.1)
# Derivative filter time constant (s)
DERIV_TAU = 0.02
# Control rate limit (unit/sec) for soft-start
CMD_RATE_LIMIT = 1.0
# Overshoot bias magnitude
OVERSHOOT_BIAS = 0.10
# —————————————————————————————————————————————————————————

class DualPID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0

    def update(self, error, measurement_dot, dt, output, output_limits):
        # Dead-band: only clamp out extremely tiny errors
        if abs(error) < DEAD_BAND:
            # allow pass-through for small errors to create overshoot
            pass

        # Integral with anti-windup: only integrate if not saturated
        u_unsat = self.Kp * error + self.Ki * self.integral + self.Kd * ((error - self.prev_error) / dt)
        # derivative raw
        d_raw = (error - self.prev_error) / dt if dt > 0 else 0.0
        # derivative filtering
        alpha = dt / (DERIV_TAU + dt)
        self.d_filtered += alpha * (d_raw - self.d_filtered)
        # anti-windup: conditional integration
        if output_limits[0] < output < output_limits[1]:
            self.integral += error * dt
        # PID calculation with filtered derivative
        u = self.Kp * error + self.Ki * self.integral + self.Kd * self.d_filtered
        # optional slight bias for overshoot
        u += OVERSHOOT_BIAS * np.sign(error)
        # saturate
        u_sat = np.clip(u, output_limits[0], output_limits[1])
        self.prev_error = error
        return u_sat


class BalanceRoverEnv:
    def __init__(self):
        xml = Path(__file__).with_name("balance_rover.xml")
        if not xml.exists():
            print("Error: balance_rover.xml not found.")
            sys.exit(1)

        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)

        # Dual PID controllers
        self.pid_ang = DualPID(Kp_ang, Ki_ang, Kd_ang)
        self.pid_vel = DualPID(Kp_vel, Ki_vel, Kd_vel)
        # disturbance estimate
        self.dist_est = 0.0

        # manual drive
        self.forward_pressed  = False
        self.backward_pressed = False
        self.drive_target     = 0.0
        self.drive_current    = 0.0
        self.u_smoothed       = 0.0
        self.wheels_enabled   = True

        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self._key_cb
        )
        self._init_button_ui()
        self.respawn()

    def _init_button_ui(self):
        self.root = tk.Tk()
        self.root.title("Dual-Loop Rover Control")
        self.root.resizable(False, False)

        # Forward/back buttons
        btn_fwd = tk.Button(self.root, text="Forward",  width=10)
        btn_bwd = tk.Button(self.root, text="Backward", width=10)
        btn_fwd.grid(row=0, column=0, padx=5, pady=5)
        btn_bwd.grid(row=0, column=1, padx=5, pady=5)

        # Angle display
        self.angle_label = tk.Label(self.root, text="Angle = 0.00°", width=20)
        self.angle_label.grid(row=1, column=0, columnspan=2, pady=(0,10))

        # Toggle wheels
        self.btn_toggle = tk.Button(self.root, text="Stop Wheels", width=12,
                                    command=self._toggle_wheels)
        self.btn_toggle.grid(row=2, column=0, columnspan=2, pady=(5,10))

        btn_fwd.bind("<ButtonPress-1>",   self._on_fwd_press)
        btn_fwd.bind("<ButtonRelease-1>", self._on_fwd_release)
        btn_bwd.bind("<ButtonPress-1>",   self._on_bwd_press)
        btn_bwd.bind("<ButtonRelease-1>", self._on_bwd_release)

    def _toggle_wheels(self):
        self.wheels_enabled = not self.wheels_enabled
        self.btn_toggle.config(text="Stop Wheels" if self.wheels_enabled else "Start Wheels")
        if not self.wheels_enabled:
            self.drive_target = 0.0
            self.drive_current = 0.0

    def respawn(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qpos[3] = np.deg2rad(10.0)
        mujoco.mj_forward(self.model, self.data)

        # reset controllers and states
        self.pid_ang.reset(); self.pid_vel.reset()
        self.dist_est = 0.0
        self.drive_target = 0.0
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.forward_pressed = False; self.backward_pressed = False

    def _on_fwd_press(self, event):
        self.forward_pressed = True; self._update_drive_target()
    def _on_fwd_release(self, event):
        self.forward_pressed = False; self._update_drive_target()
    def _on_bwd_press(self, event):
        self.backward_pressed = True; self._update_drive_target()
    def _on_bwd_release(self, event):
        self.backward_pressed = False; self._update_drive_target()

    def _update_drive_target(self):
        if not self.wheels_enabled:
            self.drive_target = 0.0
        else:
            self.drive_target = DRIVE_GAIN * (1 if self.forward_pressed else 0) \
                              - DRIVE_GAIN * (1 if self.backward_pressed else 0)

    def _key_cb(self, window, key, scancode, action, mods):
        if key==glfw.KEY_RIGHT:
            if action==glfw.PRESS: self.forward_pressed=True
            elif action==glfw.RELEASE: self.forward_pressed=False
            self._update_drive_target()
        if key==glfw.KEY_LEFT:
            if action==glfw.PRESS: self.backward_pressed=True
            elif action==glfw.RELEASE: self.backward_pressed=False
            self._update_drive_target()

    def close(self):
        self.viewer.close(); self.root.destroy()

if __name__ == '__main__':
    env = BalanceRoverEnv()
    dt = env.model.opt.timestep
    sim_time = 0.0

    while sim_time < SIM_DURATION:
        mujoco.mj_step(env.model, env.data)
        sim_time += dt

        # measure states
        theta     = env.data.qpos[3]      # angle
        theta_dot = env.data.qvel[3]      # angular velocity
        # disturbance observer: simple low-pass of measured torque error
        measured_torque = env.data.qfrc_applied[0] * env.model.actuator_gainprm[0]
        # estimate disturbance
        env.dist_est += dt * (-env.dist_est + measured_torque - env.u_smoothed)

        # outer loop: angle→velocity setpoint
        vel_sp = env.pid_ang.update(theta, theta_dot, dt,
                                    env.drive_current, (-5,5))
        # inner loop: velocity→torque
        u_ctrl = env.pid_vel.update(vel_sp - theta_dot, theta_dot, dt,
                                     env.u_smoothed, (-3,3))

        # soft-start / ramp-limit on control command
        max_delta = CMD_RATE_LIMIT * dt
        delta = u_ctrl - env.u_smoothed
        if abs(delta) > max_delta:
            env.u_smoothed += np.sign(delta) * max_delta
        else:
            env.u_smoothed = u_ctrl

        # apply manual drive addition
        err_d = env.drive_target - env.drive_current
        mdx   = DRIVE_RAMP_RATE * dt
        if abs(err_d)>mdx:
            env.drive_current += np.sign(err_d)*mdx
        else:
            env.drive_current = env.drive_target

        # final command includes disturbance compensation
        cmd = (env.u_smoothed + env.dist_est) / 3.0 if env.wheels_enabled else 0.0
        env.data.ctrl[0] = cmd
        env.data.ctrl[1] = cmd

        # update UI
        env.angle_label.config(text=f"Angle = {np.rad2deg(theta):.2f}°")
        env.viewer.sync(); env.root.update()

        if abs(theta) > FALL_THRESH:
            print(f"Fallen at t={sim_time:.3f}s (θ={theta:.3f} rad)")

    print("Simulation finished.")
    input("Press Enter to close…")
    env.close()
