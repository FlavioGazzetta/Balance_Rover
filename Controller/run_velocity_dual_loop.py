#!/usr/bin/env python3
"""
Dual‐loop velocity→tilt→torque self‐balancing rover with manual drive input.

Controls:
  • Hold L (l / L) to drive forward
  • Hold J (j / J) to drive backward

Wheel angular velocities print to console. Press Ctrl+C to exit.
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw
from mujoco import mj_id2name, mjtObj

# Joint indices in qpos/qvel: slide=0, L_spin=1, R_spin=2, hinge=3
HINGE_IDX   = 3
WHEEL_L_IDX = 1
WHEEL_R_IDX = 2

# Drive flags
drive_fwd = False
drive_bwd = False

def key_callback(*args):
    """
    Accept any call signature. Only proceed if we have
    at least 5 args: (window, key, scancode, action, mods).
    """
    global drive_fwd, drive_bwd
    if len(args) < 5:
        return
    _, key, scancode, action, mods = args[:5]
    # L or l
    if key == glfw.KEY_L:
        drive_fwd = (action == glfw.PRESS)
    # J or j
    elif key == glfw.KEY_J:
        drive_bwd = (action == glfw.PRESS)

def parse_args():
    p = argparse.ArgumentParser(description="Velocity-based dual-loop PID balance rover")
    p.add_argument('--drive-speed', type=float, default=0.1,
                   help='Manual drive speed (m/s) when key held')
    # outer velocity loop gains
    p.add_argument('--kp_vel', type=float, default=10.0)
    p.add_argument('--ki_vel', type=float, default=0.0)
    p.add_argument('--kd_vel', type=float, default=2.0)
    # inner tilt (angle) loop gains
    p.add_argument('--kp_ang', type=float, default=150.0)
    p.add_argument('--ki_ang', type=float, default=10.0)
    p.add_argument('--kd_ang', type=float, default=5.0)
    return p.parse_args()

def main():
    args = parse_args()

    # Load model & data
    xml = Path(__file__).with_name('balance_rover.xml')
    if not xml.exists():
        print('Error: balance_rover.xml not found.')
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data  = mujoco.MjData(model)

    # Find our wheel actuators
    act_names = [mj_id2name(model, mjtObj.mjOBJ_ACTUATOR, i)
                 for i in range(model.nu)]
    if 'u_L' not in act_names or 'u_R' not in act_names:
        print("Error: actuators 'u_L' and 'u_R' must be defined in balance_rover.xml.")
        sys.exit(1)
    i_L = act_names.index('u_L')
    i_R = act_names.index('u_R')

    print("Balanced rover dual-loop")
    print(f"Wheel actuators: u_L@{i_L}, u_R@{i_R}")
    print("Controls: hold L (forward) or J (backward); Ctrl+C to exit.")

    # Launch the GUI and register our key callback
    viewer = mujoco.viewer.launch_passive(
        model, data,
        key_callback=key_callback
    )
    dt = model.opt.timestep

    # PID integrators for outer (v) and inner (angle) loops
    int_v = prev_v = 0.0
    int_a = prev_a = 0.0

    try:
        step = 0
        while True:
            mujoco.mj_step(model, data)

            # Numerical stability guard
            if not np.all(np.isfinite(data.qacc)):
                print(f"\nNumerical blow-up at t={step*dt:.4f}s; exiting.")
                break

            # Velocity read & reference
            v     = data.qvel[0]
            v_ref = args.drive_speed * (1 if drive_fwd else 0) \
                  - args.drive_speed * (1 if drive_bwd else 0)

            # Outer PID: v → desired tilt (theta_ref)
            err_v = v_ref - v
            int_v += err_v * dt
            der_v = (err_v - prev_v) / dt
            prev_v = err_v
            theta_ref = (args.kp_vel * err_v
                       + args.ki_vel * int_v
                       + args.kd_vel * der_v)
            theta_ref = float(np.clip(theta_ref, -0.3, 0.3))

            # Inner PID: tilt → wheel torque
            theta = data.qpos[HINGE_IDX]
            err_a = theta_ref - theta
            int_a += err_a * dt
            der_a = (err_a - prev_a) / dt
            prev_a = err_a
            u = (args.kp_ang * err_a
                 + args.ki_ang * int_a
                 + args.kd_ang * der_a)
            u = float(np.clip(u, -3.0, 3.0))

            # Apply torque to both wheels (normalized -1..1)
            a_norm       = u / 3.0
            data.ctrl[i_L] = a_norm
            data.ctrl[i_R] = a_norm

            # Print wheel angular velocities in-place
            omega_L = data.qvel[WHEEL_L_IDX]
            omega_R = data.qvel[WHEEL_R_IDX]
            print(f"ω_L={omega_L:.2f} rad/s | ω_R={omega_R:.2f} rad/s", end='\r')

            viewer.sync()
            step += 1

    except KeyboardInterrupt:
        print("\nUser exit.")

    viewer.close()

if __name__ == '__main__':
    main()
