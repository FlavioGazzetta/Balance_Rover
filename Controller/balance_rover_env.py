#!/usr/bin/env python3
import sys, os
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer
import tkinter as tk
from collections import deque
from ctypes import CDLL, c_void_p, c_float
import numpy.ctypeslib as ctl

# 1) Load & bind C++ controller
script_dir = Path(__file__).parent.resolve()
dll_path   = script_dir/"controller_lib.dll"
if not dll_path.exists():
    sys.exit(f"Error: controller_lib.dll not found in {script_dir}")

# Windows DLL search paths
mujoco_lib = os.getenv('MUJOCO_LIB', r'C:\mujoco\mujoco210\bin')
mingw_bin  = r'C:\msys64\mingw64\bin'
if os.name=="nt" and hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(str(script_dir))
    if os.path.isdir(mujoco_lib): os.add_dll_directory(mujoco_lib)
    if os.path.isdir(mingw_bin):  os.add_dll_directory(mingw_bin)
else:
    os.environ['PATH'] += os.pathsep + str(script_dir)

lib = CDLL(str(dll_path))
lib.make_controller.restype = c_void_p
lib.free_controller.argtypes = [c_void_p]

# Bind update/controller + manual callbacks
float_arr = ctl.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
lib.update_controller.argtypes = [c_void_p, float_arr, float_arr, float_arr, float_arr]
lib.update_controller.restype  = c_float

for fn in ("on_fwd_press","on_fwd_release","on_bwd_press","on_bwd_release"):
    f = getattr(lib, fn)
    f.argtypes = [c_void_p]
    f.restype  = None

# Bind all setters
lib.set_Kp_ang.argtypes       = [c_float]
lib.set_Ki_ang.argtypes       = [c_float]
lib.set_Kd_ang.argtypes       = [c_float]
lib.set_Kp_vel.argtypes       = [c_float]
lib.set_Ki_vel.argtypes       = [c_float]
lib.set_Kd_vel.argtypes       = [c_float]
lib.set_drive_gain.argtypes   = [c_float]
lib.set_drive_ramp.argtypes   = [c_float]
lib.set_smooth_tau.argtypes   = [c_float]
lib.set_overshoot_bias.argtypes = [c_float]
lib.set_overshoot_rate.argtypes = [c_float]
lib.set_drift_gain.argtypes   = [c_float]
lib.set_drift_window.argtypes = [c_float]

# 2) Constants
DRIFT_WINDOW = 5.0

# 3) Env
class BalanceRoverEnv:
    def __init__(self):
        xml = script_dir/"balance_rover.xml"
        if not xml.exists():
            sys.exit("Error: XML not found")
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)
        self.dt    = self.model.opt.timestep

        # sensors & slide joint
        self.acc_id  = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,"pole_accel")
        self.gyro_id = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,"pole_gyro")
        sid = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,"slide")
        self.slide_dof = self.model.jnt_dofadr[sid]

        self.ctrl = lib.make_controller()
        self.drift_buffer = deque(maxlen=int(DRIFT_WINDOW/self.dt))
        self.initial_angle = 0.0

        self.viewer = mujoco.viewer.launch_passive(self.model,self.data,
                                                   key_callback=lambda *a: None)
        self._build_ui()
        self.respawn()

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("Balance Rover - Tune All Params")
        self.root.resizable(False, False)

        params = [
            ("InitAngle (°)", None,            "0.0"),
            ("Kp_ang",        lib.set_Kp_ang,   "30.0"),
            ("Ki_ang",        lib.set_Ki_ang,   "10.0"),
            ("Kd_ang",        lib.set_Kd_ang,    "0.5"),
            ("Kp_vel",        lib.set_Kp_vel,   "20.0"),
            ("Ki_vel",        lib.set_Ki_vel,    "0.5"),
            ("Kd_vel",        lib.set_Kd_vel,    "0.1"),
            ("DriveGain",     lib.set_drive_gain,"0.4"),
            ("DriveRamp",     lib.set_drive_ramp,"0.2"),
            ("SmoothTau",     lib.set_smooth_tau,"4.0"),
            ("OvershootBias", lib.set_overshoot_bias,"0.10"),
            ("OvershootRate", lib.set_overshoot_rate,"0.05"),
            ("DriftGain",     lib.set_drift_gain,"-0.4"),
            ("DriftWindow",   lib.set_drift_window,"5.0"),
        ]
        self.entries = {}
        for i,(name,setter,default) in enumerate(params):
            tk.Label(self.root,text=name).grid(row=i,column=0,sticky="e")
            e = tk.Entry(self.root,width=8); e.insert(0,default)
            e.grid(row=i,column=1,sticky="w")
            self.entries[name] = (e,setter)

        tk.Button(self.root,text="Update Params",command=self._update_params)\
          .grid(row=len(params),column=0,columnspan=2,pady=(5,5))
        tk.Button(self.root,text="Respawn",command=self.respawn)\
          .grid(row=len(params)+1,column=0,columnspan=2,pady=(0,10))

        self.status = tk.Label(self.root,text="",width=40,justify="left")
        self.status.grid(row=len(params)+2,column=0,columnspan=2,pady=(0,10))

        btn_f = tk.Button(self.root,text="Forward",width=10)
        btn_b = tk.Button(self.root,text="Backward",width=10)
        btn_f.grid(row=0,column=2,padx=5,pady=5)
        btn_b.grid(row=1,column=2,padx=5,pady=5)
        btn_f.bind("<ButtonPress-1>", lambda e: lib.on_fwd_press(self.ctrl))
        btn_f.bind("<ButtonRelease-1>",lambda e: lib.on_fwd_release(self.ctrl))
        btn_b.bind("<ButtonPress-1>", lambda e: lib.on_bwd_press(self.ctrl))
        btn_b.bind("<ButtonRelease-1>",lambda e: lib.on_bwd_release(self.ctrl))

    def _update_params(self):
        for name,(entry,setter) in self.entries.items():
            try: val=float(entry.get())
            except: continue
            if name=="InitAngle (°)":
                self.initial_angle = val
            elif setter:
                setter(val)

    def respawn(self):
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[:] = 0; self.data.qvel[:] = 0; self.data.ctrl[:] = 0
        hj = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,"hinge")
        self.data.qpos[self.model.jnt_qposadr[hj]] = np.deg2rad(self.initial_angle)
        mujoco.mj_forward(self.model,self.data)
        lib.free_controller(self.ctrl)
        self.ctrl = lib.make_controller()
        self.drift_buffer.clear()

    def run(self):
        dt_arr = np.array([self.dt],dtype=np.float32)
        while True:
            mujoco.mj_step(self.model,self.data)
            sd    = self.data.sensordata
            accel = np.array(sd[self.acc_id:self.acc_id+3],dtype=np.float32)
            gyro  = np.array(sd[self.gyro_id:self.gyro_id+3],dtype=np.float32)
            slide = np.array([self.data.qvel[self.slide_dof]],dtype=np.float32)

            cmd = float(lib.update_controller(self.ctrl, accel, gyro, slide, dt_arr))

            self.drift_buffer.append(slide[0])
            drift = sum(self.drift_buffer)/len(self.drift_buffer)

            self.data.ctrl[0]=cmd; self.data.ctrl[1]=cmd
            self.status.config(text=(
                f"InitAng= {self.initial_angle:.1f}°\n"
                f"Cmd    = {cmd:.3f}\n"
                f"Drift  = {drift:.4f} m/s"
            ))
            self.viewer.sync(); self.root.update()

    def close(self):
        lib.free_controller(self.ctrl)
        self.viewer.close(); self.root.destroy()

if __name__=="__main__":
    try:
        BalanceRoverEnv().run()
    except tk.TclError:
        pass
