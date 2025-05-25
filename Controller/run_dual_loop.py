#!/usr/bin/env python3
"""
Dual-loop PID + visual wheel spin for the balance rover.
Outer: position → desired tilt; Inner: tilt → slide force.
"""

import sys
from pathlib import Path
import argparse

import numpy as np
import mujoco
import mujoco.viewer

# Joint indices
SLIDE_IDX = 0    # slide
HINGE_IDX = 3    # pendulum hinge
WHEEL_L   = 1    # wheel_L hinge
WHEEL_R   = 2    # wheel_R hinge

# Default PID gains
DEFAULTS = {
    'kp_pos': 10.0, 'ki_pos': 0.0,  'kd_pos': 1.0,
    'kp_ang':150.0, 'ki_ang': 5.0,  'kd_ang': 2.0,
}

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--kp_pos', type=float, default=DEFAULTS['kp_pos'])
    p.add_argument('--ki_pos', type=float, default=DEFAULTS['ki_pos'])
    p.add_argument('--kd_pos', type=float, default=DEFAULTS['kd_pos'])
    p.add_argument('--kp_ang', type=float, default=DEFAULTS['kp_ang'])
    p.add_argument('--ki_ang', type=float, default=DEFAULTS['ki_ang'])
    p.add_argument('--kd_ang', type=float, default=DEFAULTS['kd_ang'])
    p.add_argument('--max-steps', type=int,   default=200_000)
    p.add_argument('--fall-thresh', type=float, default=np.pi*0.9)
    return p.parse_args()

def main():
    args = parse_args()

    # load model
    xml = Path(__file__).with_name('balance_rover.xml')
    if not xml.exists():
        print("Missing balance_rover.xml"); sys.exit(1)
    model = mujoco.MjModel.from_xml_path(str(xml))
    data  = mujoco.MjData(model)

    # launch GUI
    viewer = mujoco.viewer.launch_passive(model, data)

    dt = model.opt.timestep
    i_pos, p_pos = 0.0, 0.0
    i_ang, p_ang = 0.0, 0.0
    x_ref = 0.0

    for step in range(args.max_steps):
        mujoco.mj_step(model, data)

        # blow-up check
        if not np.all(np.isfinite(data.qacc)):
            print("Numerical instability."); break

        # states
        x   = data.qpos[SLIDE_IDX]
        th  = data.qpos[HINGE_IDX]

        # outer PID: pos → tilt_ref
        e_pos = x_ref - x
        i_pos += e_pos*dt
        d_pos = (e_pos - p_pos)/dt; p_pos = e_pos
        th_ref = (args.kp_pos*e_pos + args.ki_pos*i_pos + args.kd_pos*d_pos)
        th_ref = np.clip(th_ref, -0.3, 0.3)

        # inner PID: tilt error → force u
        e_ang = th_ref - th
        i_ang += e_ang*dt
        d_ang = (e_ang - p_ang)/dt; p_ang = e_ang
        u     = args.kp_ang*e_ang + args.ki_ang*i_ang + args.kd_ang*d_ang
        u     = float(np.clip(u, -5.0, 5.0))

        # apply directly to slide joint
        data.ctrl[0] = u

        # spin wheels visually: theta_wheel = slide_pos / wheel_radius
        wheel_radius = 0.06
        angle = data.qpos[SLIDE_IDX] / wheel_radius
        data.qpos[WHEEL_L] = angle
        data.qpos[WHEEL_R] = angle

        # render
        viewer.sync()

        if abs(th) > args.fall_thresh:
            print(f"Fell at t={step*dt:.3f}s, θ={th:.3f}")
            break

    input("Done. Press Enter to close…")
    viewer.close()

if __name__=='__main__':
    main()
