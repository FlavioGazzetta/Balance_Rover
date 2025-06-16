#!/usr/bin/env python3
"""
Controller logic for BalanceRover:
- Dual-loop PID (angle + velocity)
- Drift/movement handling (automatic drift correction, manual drive, smoothing)
- Anti-windup, derivative filtering, dead-band, bias, rate limits, etc.
"""

import sys
from pathlib import Path
import numpy as np
from collections import deque

# —————————————————————————————————————————————————————————
# INITIAL TUNING PARAMETERS (live-tunable via UI)
# Outer loop (angle) gains
Kp_ang, Ki_ang, Kd_ang = 30.0, 10.0, 0.5
# Inner loop (velocity) gains
Kp_vel, Ki_vel, Kd_vel = 20.0, 0.5, 0.1

DRIVE_GAIN      = 0.4    # N·m per button/key press
DRIVE_RAMP_RATE = 0.2    # N·m per second
SMOOTH_TAU      = 4      # seconds for exp smoothing
FALL_THRESH     = np.pi * 0.9
SIM_DURATION    = 600.0  # seconds

# Dead-band threshold (rad)
DEAD_BAND = np.deg2rad(0.1)
# Derivative filter time constant (s)
DERIV_TAU = 0.02
# Control rate limit (unit/sec) for soft-start
CMD_RATE_LIMIT = 1.0
# Overshoot bias magnitude
OVERSHOOT_BIAS = 0.10

# Drift & manual drive parameters
INITIAL_ANGLE   = 10.0    # deg for respawn routines
OVERSHOOT_RATE  = 0.05    # manual drive decay (N·m per sec)
DRIFT_WINDOW    = 5.0     # seconds
DRIFT_GAIN      = -0.4    # N·m per (m/s) drift
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
        # Dead-band: allow small errors to pass
        if abs(error) < DEAD_BAND:
            pass

        # raw derivative
        d_raw = (error - self.prev_error) / dt if dt > 0 else 0.0
        alpha = dt / (DERIV_TAU + dt)
        self.d_filtered += alpha * (d_raw - self.d_filtered)

        # anti-windup integration
        if output_limits[0] < output < output_limits[1]:
            self.integral += error * dt

        # PID with filtered derivative
        u = self.Kp * error + self.Ki * self.integral + self.Kd * self.d_filtered
        # slight overshoot bias
        u += OVERSHOOT_BIAS * np.sign(error)
        # saturate
        u_sat = np.clip(u, output_limits[0], output_limits[1])
        self.prev_error = error
        return u_sat

class RoverController:
    def __init__(self, timestep):
        # PID for pole balance
        self.pid = DualPID(Kp_ang, Ki_ang, Kd_ang)
        # Manual & drift state
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.manual_active = False
        # Drift buffer
        max_len = int(DRIFT_WINDOW / timestep)
        self.drift_buffer = deque(maxlen=max_len)

    def reset(self):
        # Reset controllers and state
        self.pid.reset()
        self.drive_current = 0.0
        self.u_smoothed = 0.0
        self.manual_active = False
        self.drift_buffer.clear()

    def on_fwd_press(self):
        self.manual_active = True
        self.drive_current += DRIVE_GAIN

    def on_fwd_release(self):
        self.manual_active = False

    def on_bwd_press(self):
        self.manual_active = True
        self.drive_current -= DRIVE_GAIN

    def on_bwd_release(self):
        self.manual_active = False

    def step(self, theta, theta_dot, slide_vel, dt):
        # Buffer slide velocity
        self.drift_buffer.append(slide_vel)
        v_avg = sum(self.drift_buffer) / len(self.drift_buffer)

        # Automatic drift correction
        if not self.manual_active:
            self.drive_current = -v_avg * DRIFT_GAIN

        # Manual drive decay / overshoot bounce
        if self.drive_current != 0.0:
            dec = OVERSHOOT_RATE * dt
            if abs(self.drive_current) > dec:
                self.drive_current -= np.sign(self.drive_current) * dec
            else:
                self.drive_current = -np.sign(self.drive_current) * (dec - abs(self.drive_current))

        # PID control for pole
        u_pid = self.pid.update(theta, theta_dot, dt, self.u_smoothed, (-300000, 300000))
        u_target = u_pid + self.drive_current

        # Exponential smoothing
        self.u_smoothed += (dt / SMOOTH_TAU) * (u_target - self.u_smoothed)

        # Final command
        cmd = float(np.clip(self.u_smoothed, -np.inf, np.inf))
        return cmd, v_avg
