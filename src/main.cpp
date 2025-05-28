#include <esp32-hal-timer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "step.h"
#include <TimerInterrupt_Generic.h>
#include <deque>

// ————————————————————————————————————————————————————————————————————
// Tuned controller globals with adaptive overshoot bias
// ————————————————————————————————————————————————————————————————————
static float g_Kp_ang         = 200.0f;   // proportional gain
static float g_Ki_ang         = 10.0f;    // integral gain
static float g_Kd_ang         = 200.0f;   // derivative gain

static const float g_OVERSHOOT_BIAS_INIT = 0.10f;  // initial bias
static const float g_OVERSHOOT_DECAY      = 0.8f;   // decay factor per overshoot
static const float g_OVERSHOOT_RECOVERY_TAU = 0.5f; // s for bias to recover

static float g_DRIVE_GAIN     = 0.4f;
static float g_SMOOTH_TAU     = 1.0f;
static float g_OVERSHOOT_RATE = 0.05f;
static float g_DRIFT_WINDOW   = 1.0f;
static float g_DRIFT_GAIN     = -0.4f;

// constants
static constexpr float Pi        = 3.14159265358979323846f;
static constexpr float DERIV_TAU = 0.02f;

// —————————————————————————————————————————————————————————
// Dual‐loop PID with adaptive overshoot bias
// —————————————————————————————————————————————————————————
struct DualPID {
    float Kp, Ki, Kd;
    float integral, prev_error, d_filtered;
    float overshoot_bias;

    DualPID()
     : Kp(g_Kp_ang), Ki(g_Ki_ang), Kd(g_Kd_ang),
       integral(0), prev_error(0), d_filtered(0),
       overshoot_bias(g_OVERSHOOT_BIAS_INIT)
    {}

    float update(float error, float dt, float min_u, float max_u) {
        // derivative filter
        float d_raw = dt > 0 ? (error - prev_error)/dt : 0;
        float alpha = dt/(DERIV_TAU + dt);
        d_filtered += alpha * (d_raw - d_filtered);

        // detect overshoot: error sign changed
        bool overshoot = (error > 0 && prev_error < 0) || (error < 0 && prev_error > 0);
        if (overshoot) {
            // decay bias immediately
            overshoot_bias *= g_OVERSHOOT_DECAY;
            // reset integral on overshoot
            integral = 0;
        } else {
            // recover bias toward initial value
            float recover_rate = dt / g_OVERSHOOT_RECOVERY_TAU;
            overshoot_bias += (g_OVERSHOOT_BIAS_INIT - overshoot_bias) * recover_rate;
        }

        // compute raw PID
        float u_raw = Kp * error
                    + Ki * integral
                    + Kd * d_filtered
                    + overshoot_bias * (error > 0 ? 1.0f : -1.0f);
        // integrate only when not overshooting and not saturated
        if (!overshoot && u_raw > min_u && u_raw < max_u) {
            integral += error * dt;
        }

        // recompute with updated integral
        u_raw = Kp * error
              + Ki * integral
              + Kd * d_filtered
              + overshoot_bias * (error > 0 ? 1.0f : -1.0f);
        
        // saturate
        float u_sat = fmax(min_u, fmin(max_u, u_raw));
        prev_error = error;
        return u_sat;
    }
};

// —————————————————————————————————————————————————————————
// RoverController: wraps DualPID + drift compensation
// —————————————————————————————————————————————————————————
struct RoverController {
    DualPID pid;
    float drive_current = 0.0f, u_smoothed = 0.0f;
    std::deque<float> drift_buffer;

    float step(float theta, float theta_dot, float accel_x, float dt) {
        // drift compensation
        drift_buffer.push_back(accel_x);
        size_t m = size_t(g_DRIFT_WINDOW / dt);
        if (drift_buffer.size() > m) drift_buffer.pop_front();
        float ax_avg = 0;
        for (float a : drift_buffer) ax_avg += a;
        ax_avg /= drift_buffer.size();
        drive_current = -ax_avg * g_DRIFT_GAIN;

        // manual-drive decay
        float dec = g_OVERSHOOT_RATE * dt;
        if (fabs(drive_current) > dec)
            drive_current -= copysign(dec, drive_current);
        else
            drive_current = 0;

        // PID update
        float u_pid = pid.update(theta, dt, -3e5f, 3e5f);
        float target = u_pid + drive_current;
        u_smoothed += (dt / g_SMOOTH_TAU) * (target - u_smoothed);
        return u_smoothed;
    }
};

// —————————————————————————————————————————————————
// Hardware & globals
// —————————————————————————————————————————————————
static const int STEPPER_INTERVAL_US = 100;
static const int dirPin_r = 16, stepPin_r = 17;
static const int dirPin_l = 4,  stepPin_l = 14;
static const int STEPPER_EN_PIN = 15, togglePin = 19;
static step right_stepper(STEPPER_INTERVAL_US, stepPin_r, dirPin_r);
static step left_stepper (STEPPER_INTERVAL_US, stepPin_l, dirPin_l);
static ESP32Timer stepTimer(3);
static Adafruit_MPU6050 mpu;
static RoverController ctrl;
unsigned long previous_time = 0;
float tilt_bias = 0.0f;

// ISR: step motors & toggle pin
bool IRAM_ATTR onTimer(void*) {
    right_stepper.runStepper();
    left_stepper .runStepper();
    static bool t = false;
    digitalWrite(togglePin, t);
    t = !t;
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Balance Robot — adaptive overshoot bias enabled");

    Wire.begin(21, 22);
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    pinMode(dirPin_r, OUTPUT);
    pinMode(stepPin_r, OUTPUT);
    pinMode(dirPin_l, OUTPUT);
    pinMode(stepPin_l, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(togglePin, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);

    constexpr float ACCEL_RAD = 60.0f;
    left_stepper .setAccelerationRad(ACCEL_RAD);
    right_stepper.setAccelerationRad(ACCEL_RAD);

    if (!stepTimer.attachInterruptInterval(STEPPER_INTERVAL_US, onTimer)) {
        Serial.println("Failed to start ISR");
        while (1) delay(10);
    }

    delay(100);
    sensors_event_t ea, eg, et;
    mpu.getEvent(&ea, &eg, &et);
    float ax = ea.acceleration.x/16384.0f;
    float az = ea.acceleration.z/16384.0f;
    tilt_bias = atan2(-ax, az);
    Serial.print("tilt_bias (rad): "); Serial.println(tilt_bias, 6);

    previous_time = micros();
}

void loop() {
    unsigned long now = micros();
    float dt = (now - previous_time) * 1e-6f;
    previous_time = now;
    if (dt < 1e-4f) dt = 1e-4f;

    sensors_event_t ea, eg, et;
    mpu.getEvent(&ea, &eg, &et);
    float ax = ea.acceleration.x / 16384.0f;
    float az = ea.acceleration.z / 16384.0f;
    float gyro_y = eg.gyro.y;

    float raw       = atan2(-ax, az);
    float theta     = raw - tilt_bias;
    float theta_dot = (gyro_y/131.0f) * (Pi/180.0f);

    float u = ctrl.step(theta, theta_dot, ax, dt);
    right_stepper.setTargetSpeedRad( u);
    left_stepper .setTargetSpeedRad(-u);

    Serial.print("dt: ");Serial.print(dt,4);
    Serial.print(" raw: ");Serial.print(raw,4);
    Serial.print(" theta: ");Serial.print(theta,4);
    Serial.print(" theta_dot: ");Serial.print(theta_dot,4);
    Serial.print(" u: ");Serial.println(u,4);

    delay(1);
}