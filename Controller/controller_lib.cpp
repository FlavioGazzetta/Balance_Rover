#include <cstdint>
#include <cstdlib>
#include <deque>
#include <cmath>
#include <cstring>

extern "C" {

// ————————————————————————————————————————————————————————————————————
// Runtime‑tunable globals (defaults matching your Python PID)
// ————————————————————————————————————————————————————————————————————
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

// static constexpr things left unchanged:
static constexpr float PI        = 3.14159265358979323846f;
static constexpr float DEAD_BAND = (PI/180.0f) * 0.1f;
static constexpr float DERIV_TAU = 0.02f;

// —————————————————————————————————————————————————————————
// Dual‑loop PID implementation
// —————————————————————————————————————————————————————————
struct DualPID {
    float Kp, Ki, Kd;
    float integral, prev_error, d_filtered;

    DualPID()
     : Kp(g_Kp_ang), Ki(g_Ki_ang), Kd(g_Kd_ang),
       integral(0), prev_error(0), d_filtered(0)
    {}

    void reset() {
        Kp = g_Kp_ang; Ki = g_Ki_ang; Kd = g_Kd_ang;
        integral = prev_error = d_filtered = 0;
    }

    float update(float error, float /*error_dot*/, float dt,
                 float output, float min_u, float max_u)
    {
        if (std::fabs(error) < DEAD_BAND) {
            // no change
        }
        float d_raw = dt > 0 ? (error - prev_error) / dt : 0;
        float alpha = dt / (DERIV_TAU + dt);
        d_filtered += alpha * (d_raw - d_filtered);

        if (output > min_u && output < max_u)
            integral += error * dt;

        float u = Kp * error + Ki * integral + Kd * d_filtered
                + g_OVERSHOOT_BIAS * (error > 0 ? 1.0f : -1.0f);

        float u_sat = std::fmax(min_u, std::fmin(max_u, u));
        prev_error = error;
        return u_sat;
    }
};

// —————————————————————————————————————————————————————————
// RoverController: wraps DualPID + drift/manual drive logic
// —————————————————————————————————————————————————————————
struct RoverController {
    DualPID pid;
    float drive_current;
    float u_smoothed;
    bool manual_active;
    std::deque<float> drift_buffer;

    RoverController()
     : drive_current(0), u_smoothed(0), manual_active(false)
    {}

    void reset() {
        pid.reset();
        drive_current  = 0;
        u_smoothed     = 0;
        manual_active  = false;
        drift_buffer.clear();
    }

    void on_fwd_press()   { manual_active = true;  drive_current += g_DRIVE_GAIN; }
    void on_fwd_release() { manual_active = false; }
    void on_bwd_press()   { manual_active = true;  drive_current -= g_DRIVE_GAIN; }
    void on_bwd_release() { manual_active = false; }

    // Now uses accel_x for drift estimation instead of slide velocity
    float step(float theta, float theta_dot, float accel_x, float dt) {
        // drift buffer with acceleration
        drift_buffer.push_back(accel_x);
        size_t m = size_t(g_DRIFT_WINDOW / dt);
        if (drift_buffer.size() > m) drift_buffer.pop_front();

        float ax_avg = 0;
        for (auto &a : drift_buffer) ax_avg += a;
        ax_avg /= drift_buffer.size();

        if (!manual_active)
            drive_current = -ax_avg * g_DRIFT_GAIN;

        // manual‑drive decay
        if (drive_current != 0) {
            float dec = g_OVERSHOOT_RATE * dt;
            if (std::fabs(drive_current) > dec)
                drive_current -= (drive_current > 0 ? 1 : -1) * dec;
            else
                drive_current = -(drive_current > 0 ? 1 : -1) * (dec - std::fabs(drive_current));
        }

        // PID update (uses angle gains)
        float u_pid = pid.update(theta, theta_dot, dt, u_smoothed, -3e5f, 3e5f);
        float target = u_pid + drive_current;
        u_smoothed += (dt / g_SMOOTH_TAU) * (target - u_smoothed);
        return u_smoothed;
    }
};

static RoverController* ctrl = nullptr;

// —————————————————————————————————————————————————————————
// C API
// —————————————————————————————————————————————————————————
void* make_controller() {
    delete ctrl;
    ctrl = new RoverController();
    return ctrl;
}

void free_controller(void* ptr) {
    delete static_cast<RoverController*>(ptr);
    ctrl = nullptr;
}

/// update_controller:
///   accel:     float[3]
///   gyro:      float[3]
///   slide_vel: float[1] (unused, now accel_x used)
///   dt:        float[1]
float update_controller(void* ptr,
                        float* accel,
                        float* gyro,
                        float* /*slide_vel*/, 
                        float* dt_ptr)
{
    float ax = accel[0], az = accel[2];
    float gy = gyro[1];
    float dt = dt_ptr[0];

    float theta     = std::atan2(-ax, az);
    float theta_dot = gy;

    return static_cast<RoverController*>(ptr)
          ->step(theta, theta_dot, ax, dt);
}

// Manual‑drive
void on_fwd_press(void* ptr)   { ctrl->on_fwd_press(); }
void on_fwd_release(void* ptr) { ctrl->on_fwd_release(); }
void on_bwd_press(void* ptr)   { ctrl->on_bwd_press(); }
void on_bwd_release(void* ptr) { ctrl->on_bwd_release(); }

// Setters
void set_Kp_ang(float v)        { g_Kp_ang = v; }
void set_Ki_ang(float v)        { g_Ki_ang = v; }
void set_Kd_ang(float v)        { g_Kd_ang = v; }

void set_Kp_vel(float v)        { g_Kp_vel = v; }
void set_Ki_vel(float v)        { g_Ki_vel = v; }
void set_Kd_vel(float v)        { g_Kd_vel = v; }

void set_drive_gain(float v)    { g_DRIVE_GAIN = v; }
void set_drive_ramp(float v)    { g_DRIVE_RAMP = v; }
void set_smooth_tau(float v)    { g_SMOOTH_TAU = v; }

void set_overshoot_bias(float v){ g_OVERSHOOT_BIAS = v; }
void set_overshoot_rate(float v){ g_OVERSHOOT_RATE = v; }

void set_drift_gain(float v)    { g_DRIFT_GAIN = v; }
void set_drift_window(float v)  { g_DRIFT_WINDOW = v; }

} // extern "C"
