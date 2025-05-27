#pragma once

extern "C" {
  // create / destroy
  void*  make_controller();
  void   free_controller(void*);

  // step the dual-loop PID + drift controller
  float  update_controller(void*,
                           float accel[3],
                           float gyro[3],
                           float slide_vel[1],
                           float dt[1]);

  // manual-drive callbacks
  void   on_fwd_press(void*);
  void   on_fwd_release(void*);
  void   on_bwd_press(void*);
  void   on_bwd_release(void*);

  // live-tunable setters
  void   set_Kp_ang(float);
  void   set_Ki_ang(float);
  void   set_Kd_ang(float);
  void   set_Kp_vel(float);
  void   set_Ki_vel(float);
  void   set_Kd_vel(float);
  void   set_drive_gain(float);
  void   set_drive_ramp(float);
  void   set_smooth_tau(float);
  void   set_overshoot_bias(float);
  void   set_overshoot_rate(float);
  void   set_drift_gain(float);
  void   set_drift_window(float);
}
