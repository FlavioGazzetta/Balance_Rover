// controller_lib.cpp
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <cmath>
#include <cstring>

extern "C" {

// ————————————————————————————————————————————————————————————————————
// Compile-time constants (same defaults as in your Python PID)
// ————————————————————————————————————————————————————————————————————
static constexpr float PI = 3.14159265358979323846f;

// Outer loop (angle) gains
static constexpr float Kp_ang = 30.0f;
static constexpr float Ki_ang = 10.0f;
static constexpr float Kd_ang = 0.5f;

// Inner loop (velocity) gains
static constexpr float Kp_vel = 20.0f;
static constexpr float Ki_vel = 0.5f;
static constexpr float Kd_vel = 0.1f;

// Drive & smoothing parameters
static constexpr float DRIVE_GAIN      = 0.4f;
static constexpr float DRIVE_RAMP_RATE = 0.2f;
static constexpr float SMOOTH_TAU      = 4.0f;

// Simulation thresholds & limits
static constexpr float FALL_THRESH  = PI * 0.9f;
static constexpr float SIM_DURATION = 600.0f;

// Dead-band (rad) & derivative filter
static constexpr float DEAD_BAND = (PI/180.0f) * 0.1f;
static constexpr float DERIV_TAU = 0.02f;

// Overshoot bias & rate limit
static constexpr float OVERSHOOT_BIAS = 0.10f;
static constexpr float OVERSHOOT_RATE = 0.05f;

// Drift correction
static constexpr float DRIFT_WINDOW = 5.0f;
static constexpr float DRIFT_GAIN   = -0.4f;

// Initial respawn angle (rad)
static constexpr float INITIAL_ANGLE = 10.0f * (PI/180.0f);

// —————————————————————————————————————————————————————————
// Dual‐loop PID implementation
// —————————————————————————————————————————————————————————
struct DualPID {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float d_filtered;

    DualPID(float _Kp, float _Ki, float _Kd)
     : Kp(_Kp), Ki(_Ki), Kd(_Kd),
       integral(0.0f), prev_error(0.0f), d_filtered(0.0f)
    {}

    void reset() {
        integral   = 0.0f;
        prev_error = 0.0f;
        d_filtered = 0.0f;
    }

    float update(float error, float /*error_dot*/, float dt, float output, float min_u, float max_u) {
        // Dead-band pass
        if (std::fabs(error) < DEAD_BAND) {
            // no change to integral/derivative
        }

        // raw derivative & first-order filter
        float d_raw = dt > 0.0f ? (error - prev_error) / dt : 0.0f;
        float alpha = dt / (DERIV_TAU + dt);
        d_filtered += alpha * (d_raw - d_filtered);

        // anti-windup integration
        if (output > min_u && output < max_u) {
            integral += error * dt;
        }

        // PID formula + small overshoot bias
        float u = Kp * error
                + Ki * integral
                + Kd * d_filtered
                + OVERSHOOT_BIAS * (error > 0.0f ? 1.0f : -1.0f);

        // saturate
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
     : pid(Kp_ang, Ki_ang, Kd_ang),
       drive_current(0.0f),
       u_smoothed(0.0f),
       manual_active(false)
    {}

    void reset() {
        pid.reset();
        drive_current = 0.0f;
        u_smoothed    = 0.0f;
        manual_active = false;
        drift_buffer.clear();
    }

    // manual drive callbacks
    void on_fwd_press()   { manual_active = true;  drive_current += DRIVE_GAIN; }
    void on_fwd_release() { manual_active = false; }
    void on_bwd_press()   { manual_active = true;  drive_current -= DRIVE_GAIN; }
    void on_bwd_release() { manual_active = false; }

    // main step: returns control command
    float step(float theta, float theta_dot, float slide_vel, float dt) {
        // update drift buffer & compute average
        drift_buffer.push_back(slide_vel);
        size_t max_len = static_cast<size_t>(DRIFT_WINDOW / dt);
        if (drift_buffer.size() > max_len)
            drift_buffer.pop_front();

        float v_avg = 0.0f;
        for (auto &v : drift_buffer) v_avg += v;
        v_avg = v_avg / drift_buffer.size();

        // automatic drift correction
        if (!manual_active) {
            drive_current = -v_avg * DRIFT_GAIN;
        }

        // manual‐drive decay / overshoot bounce
        if (drive_current != 0.0f) {
            float dec = OVERSHOOT_RATE * dt;
            if (std::fabs(drive_current) > dec) {
                drive_current -= (drive_current > 0.0f ? 1.0f : -1.0f) * dec;
            } else {
                drive_current = -(drive_current > 0.0f ? 1.0f : -1.0f) * (dec - std::fabs(drive_current));
            }
        }

        // outer & inner PID → target command
        float u_pid     = pid.update(theta, theta_dot, dt, u_smoothed, -3e5f, 3e5f);
        float u_target  = u_pid + drive_current;
        // exponential smoothing
        u_smoothed += (dt / SMOOTH_TAU) * (u_target - u_smoothed);

        return u_smoothed;
    }
};

// —————————————————————————————————————————————————————————
// C API: factory, destroyer, step & manual callbacks
// —————————————————————————————————————————————————————————

void* make_controller() {
    return static_cast<void*>(new RoverController());
}

void free_controller(void* ptr) {
    delete static_cast<RoverController*>(ptr);
}

float update_controller(void* ptr,
                        float* theta,
                        float* theta_dot,
                        float* slide_vel,
                        float* dt_ptr)
{
    auto *ctrl = static_cast<RoverController*>(ptr);
    return ctrl->step(theta[0], theta_dot[0], slide_vel[0], dt_ptr[0]);
}

void on_fwd_press(void* ptr) {
    static_cast<RoverController*>(ptr)->on_fwd_press();
}
void on_fwd_release(void* ptr) {
    static_cast<RoverController*>(ptr)->on_fwd_release();
}
void on_bwd_press(void* ptr) {
    static_cast<RoverController*>(ptr)->on_bwd_press();
}
void on_bwd_release(void* ptr) {
    static_cast<RoverController*>(ptr)->on_bwd_release();
}

} // extern "C"
