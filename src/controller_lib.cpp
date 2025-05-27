#include <cmath>
#include <deque>
#include "controller_lib.h"

// tuning globals
static float g_Kp_ang        = 30.0f;
static float g_Ki_ang        = 10.0f;
static float g_Kd_ang        = 0.5f;

static float g_Kp_vel        = 20.0f;
static float g_Ki_vel        = 0.5f;
static float g_Kd_vel        = 0.1f;

static float g_DRIVE_GAIN    = 0.4f;
static float g_DRIVE_RAMP    = 0.2f;
static float g_SMOOTH_TAU    = 4.0f;

static float g_OVERSHOOT_BIAS = 0.10f;
static float g_OVERSHOOT_RATE = 0.05f;

static float g_DRIFT_WINDOW  = 5.0f;
static float g_DRIFT_GAIN    = -0.4f;

// constants
static constexpr float PI        = 3.14159265358979323846f;
static constexpr float DEAD_BAND = (PI/180.0f) * 0.1f;
static constexpr float DERIV_TAU = 0.02f;

// Dual-loop PID
struct DualPID {
  float Kp,Ki,Kd, integral, prev_error, d_filtered;
  DualPID(): Kp(g_Kp_ang),Ki(g_Ki_ang),Kd(g_Kd_ang),
             integral(0),prev_error(0),d_filtered(0){}
  void reset() {
    Kp=g_Kp_ang; Ki=g_Ki_ang; Kd=g_Kd_ang;
    integral=prev_error=d_filtered=0;
  }
  float update(float error, float /*edot*/, float dt,
               float output, float min_u, float max_u)
  {
    if (std::fabs(error) < DEAD_BAND) {}
    float d_raw = dt>0 ? (error-prev_error)/dt : 0;
    float alpha = dt/(DERIV_TAU+dt);
    d_filtered += alpha*(d_raw-d_filtered);
    if (output>min_u && output<max_u)
      integral += error*dt;
    float u = Kp*error + Ki*integral + Kd*d_filtered
            + g_OVERSHOOT_BIAS*(error>0?1:-1);
    float u_sat = std::fmax(min_u,std::fmin(max_u,u));
    prev_error = error;
    return u_sat;
  }
};

// RoverController
struct RoverController {
  DualPID pid;
  float drive_current=0, u_smoothed=0;
  bool manual_active=false;
  std::deque<float> drift_buffer;
  void reset() {
    pid.reset();
    drive_current=u_smoothed=0;
    manual_active=false;
    drift_buffer.clear();
  }
  void on_fwd_press()   { manual_active=true;  drive_current += g_DRIVE_GAIN; }
  void on_fwd_release() { manual_active=false; }
  void on_bwd_press()   { manual_active=true;  drive_current -= g_DRIVE_GAIN; }
  void on_bwd_release() { manual_active=false; }

  float step(float theta, float theta_dot, float accel_x, float dt) {
    drift_buffer.push_back(accel_x);
    size_t m = size_t(g_DRIFT_WINDOW/dt);
    if (drift_buffer.size()>m) drift_buffer.pop_front();
    float ax_avg=0;
    for(auto&a:drift_buffer) ax_avg+=a;
    ax_avg/=drift_buffer.size();
    if(!manual_active) drive_current = -ax_avg*g_DRIFT_GAIN;
    if(drive_current!=0){
      float dec = g_OVERSHOOT_RATE*dt;
      if(std::fabs(drive_current)>dec)
        drive_current -= (drive_current>0?1:-1)*dec;
      else
        drive_current = -(drive_current>0?1:-1)*(dec-std::fabs(drive_current));
    }
    float u_pid = pid.update(theta,theta_dot,dt,u_smoothed,-3e5f,3e5f);
    float target = u_pid + drive_current;
    u_smoothed += (dt/g_SMOOTH_TAU)*(target - u_smoothed);
    return u_smoothed;
  }
};

static RoverController* ctrl = nullptr;

extern "C" {

void* make_controller() {
  delete ctrl;
  ctrl = new RoverController();
  return ctrl;
}

void free_controller(void* p) {
  delete static_cast<RoverController*>(p);
  ctrl = nullptr;
}

float update_controller(void* p, float accel[3], float gyro[3],
                        float slide_vel[1], float dt_arr[1])
{
  float ax = accel[0], az = accel[2], gy = gyro[1], dt = dt_arr[0];
  float theta = std::atan2(-ax,az), theta_dot = gy;
  return static_cast<RoverController*>(p)->step(theta,theta_dot,ax,dt);
}

void on_fwd_press(void*)   { ctrl->on_fwd_press(); }
void on_fwd_release(void*) { ctrl->on_fwd_release(); }
void on_bwd_press(void*)   { ctrl->on_bwd_press(); }
void on_bwd_release(void*) { ctrl->on_bwd_release(); }

#define SETTER(name, gvar) \
  void name(float v){ gvar = v; }
SETTER(set_Kp_ang,        g_Kp_ang)
SETTER(set_Ki_ang,        g_Ki_ang)
SETTER(set_Kd_ang,        g_Kd_ang)
SETTER(set_Kp_vel,        g_Kp_vel)
SETTER(set_Ki_vel,        g_Ki_vel)
SETTER(set_Kd_vel,        g_Kd_vel)
SETTER(set_drive_gain,    g_DRIVE_GAIN)
SETTER(set_drive_ramp,    g_DRIVE_RAMP)
SETTER(set_smooth_tau,    g_SMOOTH_TAU)
SETTER(set_overshoot_bias,g_OVERSHOOT_BIAS)
SETTER(set_overshoot_rate,g_OVERSHOOT_RATE)
SETTER(set_drift_gain,    g_DRIFT_GAIN)
SETTER(set_drift_window,  g_DRIFT_WINDOW)

} // extern "C"
