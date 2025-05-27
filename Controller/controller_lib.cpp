// controller_lib.cpp
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <cmath>
#include <cstdio>

extern "C" {

// ————————————————————————————————————————————————————————————————————
// Tunable constants (edit via setters at runtime)
// ————————————————————————————————————————————————————————————————————
static float Kp_ang       = 30.0f;
static float Ki_ang       = 10.0f;
static float Kd_ang       = 0.5f;
static float DRIVE_GAIN   = 0.4f;
static float OVERSHOOT    = 0.05f;
static float SMOOTH_TAU   = 4.0f;
static float DRIFT_GAIN   = -0.4f;
static float DRIFT_WINDOW = 5.0f;

// ————————————————————————————————————————————————————————————————————
// Accelerometer dead‐band threshold (in m/s²). Below this, ignore accel.
// ————————————————————————————————————————————————————————————————————
static constexpr float ACCEL_DEADBAND = 0.1f;

// —————————————————————————————————————————————————————————
// Dual‐loop PID
// —————————————————————————————————————————————————————————
struct DualPID {
    float Kp, Ki, Kd, integral, prev_error, d_filtered;
    DualPID()
      : Kp(Kp_ang), Ki(Ki_ang), Kd(Kd_ang),
        integral(0), prev_error(0), d_filtered(0) {}
    void reset() {
        Kp = Kp_ang; Ki = Ki_ang; Kd = Kd_ang;
        integral=0; prev_error=0; d_filtered=0;
    }
    float update(float err, float /*err_dot*/, float dt,
                 float out, float mn, float mx)
    {
        float d_raw = dt>0 ? (err - prev_error)/dt : 0;
        float alpha = dt/(0.02f + dt);
        d_filtered += alpha*(d_raw - d_filtered);

        if (out>mn && out<mx) integral += err*dt;

        float u = Kp*err + Ki*integral + Kd*d_filtered
                + 0.10f * (err>0 ? 1.0f : -1.0f);
        float us = std::fmax(mn, std::fmin(mx, u));
        prev_error = err;
        return us;
    }
};

// ————————————————————————————————————————————————————————————————————
// RoverController: wraps PID + drift/manual + smoothing + IMU fusion
// ————————————————————————————————————————————————————————————————————
struct RoverController {
    DualPID pid;
    float drive_gain, overshoot_rate, smooth_tau, drift_gain, drift_window;
    float u_smoothed;
    bool  manual_active;
    std::deque<float> drift_buffer;

    // instantaneous IMU outputs
    float angle;     // from accel & gyro
    float ang_rate;  // from gyro

    RoverController()
     : drive_gain(DRIVE_GAIN),
       overshoot_rate(OVERSHOOT),
       smooth_tau(SMOOTH_TAU),
       drift_gain(DRIFT_GAIN),
       drift_window(DRIFT_WINDOW),
       u_smoothed(0),
       manual_active(false),
       drift_buffer(),
       angle(0),
       ang_rate(0)
    {}

    void reset() {
        pid.reset();
        drive_gain     = DRIVE_GAIN;
        overshoot_rate = OVERSHOOT;
        smooth_tau     = SMOOTH_TAU;
        drift_gain     = DRIFT_GAIN;
        drift_window   = DRIFT_WINDOW;
        u_smoothed     = 0;
        manual_active  = false;
        drift_buffer.clear();
        angle = 0;
        ang_rate = 0;
    }

    void on_fwd_press()   { manual_active=true;  drive_gain += DRIVE_GAIN; }
    void on_fwd_release() { manual_active=false; }
    void on_bwd_press()   { manual_active=true;  drive_gain -= DRIVE_GAIN; }
    void on_bwd_release() { manual_active=false; }

    // Fuse IMU: accel→angle (with dead-band), gyro→rate
    void fuse_imu(const float* accel, const float* gyro, float /*dt*/) {
        float ax = accel[0], az = accel[2];
        float mag = std::fabs(ax) + std::fabs(az);
        if (mag >= ACCEL_DEADBAND) {
            angle = std::atan2(-ax, -az);
        }
        ang_rate = gyro[1];
    }

    // Core control step
    float step(float slide_vel, float dt) {
        // drift correction
        drift_buffer.push_back(slide_vel);
        size_t m = size_t(drift_window/dt);
        if (drift_buffer.size() > m) drift_buffer.pop_front();

        float v_avg = 0;
        for (auto &v : drift_buffer) v_avg += v;
        v_avg /= drift_buffer.size();

        if (!manual_active) {
            drive_gain = -v_avg * drift_gain;
        }

        if (drive_gain != 0.0f) {
            float dec = overshoot_rate * dt;
            if (std::fabs(drive_gain) > dec)
                drive_gain -= (drive_gain > 0 ? 1 : -1) * dec;
            else
                drive_gain = -(drive_gain > 0 ? 1 : -1) * (dec - std::fabs(drive_gain));
        }

        float u_pid = pid.update(-angle, ang_rate, dt, u_smoothed, -3e5f, 3e5f);
        float ut    = u_pid + drive_gain;
        u_smoothed += (dt / smooth_tau) * (ut - u_smoothed);
        return u_smoothed;
    }
};

static RoverController* ctrl = nullptr;

// ————————————————————————————————————————————————————————————————————
// C API
// ————————————————————————————————————————————————————————————————————

void* make_controller() {
    delete ctrl;
    ctrl = new RoverController();
    return ctrl;
}

void free_controller(void* p) {
    delete static_cast<RoverController*>(p);
    ctrl = nullptr;
}

// setters
void set_Kp_ang(float v)       { Kp_ang = v; }
void set_Ki_ang(float v)       { Ki_ang = v; }
void set_Kd_ang(float v)       { Kd_ang = v; }
void set_drive_gain(float v)   { DRIVE_GAIN = v; }
void set_overshoot(float v)    { OVERSHOOT = v; }
void set_smooth_tau(float v)   { SMOOTH_TAU = v; }
void set_drift_gain(float v)   { DRIFT_GAIN = v; }
void set_drift_window(float v) { DRIFT_WINDOW = v; }

// manual‐drive
void on_fwd_press(void* p)    { ctrl->on_fwd_press(); }
void on_fwd_release(void* p)  { ctrl->on_fwd_release(); }
void on_bwd_press(void* p)    { ctrl->on_bwd_press(); }
void on_bwd_release(void* p)  { ctrl->on_bwd_release(); }

// IMU fusion & accessors
void controller_fuse_imu(void* p, float* accel, float* gyro, float* dt) {
    ctrl->fuse_imu(accel, gyro, dt[0]);
}
float controller_get_theta(void* p)     { return ctrl->angle; }
float controller_get_theta_dot(void* p) { return ctrl->ang_rate; }

// full control step
float update_controller(void* p,
                        float* accel, float* gyro,
                        float* slide_vel, float* dt)
{
    ctrl->fuse_imu(accel, gyro, dt[0]);
    float cmd = ctrl->step(slide_vel[0], dt[0]);
    std::printf("θ=%.3f rad, θ̇=%.3f rad/s\n", ctrl->angle, ctrl->ang_rate);
    return cmd;
}

} // extern "C"
