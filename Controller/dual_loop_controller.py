#!/usr/bin/env python3
"""
Dual-Loop PID Controller for Balance Rover

Implements an outer loop for position (cart) control and an inner loop for
pendulum angle control. The outer loop computes a desired pendulum angle based
on cart position error; the inner loop computes wheel torques to track that angle.

Usage:
    python dual_loop_controller.py [--Kp_pos Kp] [--Kd_pos Kd]
                                  [--Kp_ang Kp] [--Ki_ang Ki] [--Kd_ang Kd]
                                  [--steps N] [--initial-tip TIP]
"""
import sys
import argparse
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# Joint indices in qpos/qvel:
# 0: slide, 1: L_spin, 2: R_spin, 3: hinge
IDX_SLIDE = 0
IDX_HINGE = 3


def parse_args():
    parser = argparse.ArgumentParser(description="Dual-Loop PID for Balance Rover")
    # Outer loop gains (position -> desired angle)
    parser.add_argument("--Kp_pos", type=float, default=2.0, help="Position P gain")
    parser.add_argument("--Kd_pos", type=float, default=1.0, help="Position D gain")
    # Inner loop gains (angle -> torque)
    parser.add_argument("--Kp_ang", type=float, default=200.0, help="Angle P gain")
    parser.add_argument("--Ki_ang", type=float, default=10.0,  help="Angle I gain")
    parser.add_argument("--Kd_ang", type=float, default=20.0,  help="Angle D gain")
    parser.add_argument("--steps", type=int, default=200_000, help="Max simulation steps")
    parser.add_argument("--initial-tip", type=float, default=0.02, help="Initial pendulum tip (rad)")
    return parser.parse_args()


def main():
    args = parse_args()

    # Load model
    xml = Path(__file__).with_name("balance_rover.xml")
    if not xml.exists():
        print(f"Error: {xml.name} not found.")
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data  = mujoco.MjData(model)

    # Initial small tip
    data.qpos[IDX_HINGE] = args.initial_tip

    # Launch viewer
    viewer = mujoco.viewer.launch_passive(model, data)

    # Time step
    dt = model.opt.timestep

    # Inner loop integrator state
    integral_ang = 0.0
    prev_err_ang = 0.0

    for step in range(args.steps):
        # Advance sim
        mujoco.mj_step(model, data)

        # Crash detection
        if not np.all(np.isfinite(data.qacc)):
            print(f"Simulation unstable at step {step}, aborting.")
            break

        # Read states
        x      = data.qpos[IDX_SLIDE]
        x_dot  = data.qvel[IDX_SLIDE]
        theta  = data.qpos[IDX_HINGE]
        theta_dot = data.qvel[IDX_HINGE]

        # Outer loop: desired angle = -Kp_pos*x - Kd_pos*x_dot
        theta_ref = -(args.Kp_pos * x + args.Kd_pos * x_dot)
        # clamp desired angle small
        theta_ref = np.clip(theta_ref, -0.2, 0.2)

        # Inner loop: angle error
        err_ang = theta - theta_ref
        integral_ang += err_ang * dt
        derivative_ang = (err_ang - prev_err_ang) / dt
        prev_err_ang = err_ang

        # PID torque
        u = -(args.Kp_ang * err_ang + args.Ki_ang * integral_ang + args.Kd_ang * derivative_ang)
        u = float(np.clip(u, -3.0, 3.0))

        # Apply to both wheels
        cmd = u / 3.0  # normalize
        data.ctrl[0] = cmd
        data.ctrl[1] = cmd

        # Render
        viewer.sync()

        # Check fall
        if abs(theta) > np.pi*0.9:
            t = step * dt
            print(f"Fallen at time {t:.3f}s, theta={theta:.3f} rad")
            break

    # Wait before exit
    input("Done. Press Enter to close.")
    viewer.close()


if __name__ == "__main__":
    main()
