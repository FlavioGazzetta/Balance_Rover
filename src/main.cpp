#include <esp32-hal-timer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "step.h"
#include <TimerInterrupt_Generic.h>
#include <deque>

// ————————————————————————————————————————————————————————————————————
// Tuned controller globals
// ————————————————————————————————————————————————————————————————————
static float g_Kp_ang         = 200.0f;   // increased for faster response
static float g_Ki_ang         = 10.0f;   // slight integral bump
static float g_Kd_ang         = 200.0f;    // much more damping

static float g_DRIVE_GAIN     = 0.4f;
static float g_SMOOTH_TAU     = 1.0f;    // faster output smoothing
static float g_OVERSHOOT_BIAS = 0.10f;
static float g_OVERSHOOT_RATE = 0.05f;

static float g_DRIFT_WINDOW   = 1.0f;    // shorter drift buffer
static float g_DRIFT_GAIN     = -0.4f;

// constants
static constexpr float Pi        = 3.14159265358979323846f;
static constexpr float DEAD_BAND = (PI/180.0f) * 0.1f;
static constexpr float DERIV_TAU = 0.02f;

// —————————————————————————————————————————————————————————
// Dual‐loop PID implementation
// —————————————————————————————————————————————————————————
struct DualPID {
    float Kp, Ki, Kd;
    float integral, prev_error, d_filtered;

    DualPID()
     : Kp(g_Kp_ang), Ki(g_Ki_ang), Kd(g_Kd_ang),
       integral(0), prev_error(0), d_filtered(0)
    {}

    float update(float error, float /*error_dot*/, float dt,
                 float output, float min_u, float max_u)
    {
        // derivative filter
        float d_raw = dt > 0 ? (error - prev_error)/dt : 0;
        float alpha = dt/(DERIV_TAU + dt);
        d_filtered += alpha*(d_raw - d_filtered);

        // integral (only when not saturated)
        if (output > min_u && output < max_u)
            integral += error * dt;

        // PID plus bias
        float u = Kp*error
                + Ki*integral
                + Kd*d_filtered
                + g_OVERSHOOT_BIAS*(error>0 ? 1.0f : -1.0f);

        // saturate
        float u_sat = fmax(min_u, fmin(max_u, u));
        prev_error = error;
        return u_sat;
    }
};

// —————————————————————————————————————————————————————————
// RoverController: wraps DualPID + drift compensation
// —————————————————————————————————————————————————————————
struct RoverController {
    DualPID pid;
    float drive_current = 0, u_smoothed = 0;
    std::deque<float> drift_buffer;

    float step(float theta, float theta_dot, float accel_x, float dt) {
        // drift buffer
        drift_buffer.push_back(accel_x);
        size_t m = size_t(g_DRIFT_WINDOW/dt);
        if (drift_buffer.size()>m) drift_buffer.pop_front();

        float ax_avg = 0;
        for (auto &a : drift_buffer) ax_avg += a;
        ax_avg /= drift_buffer.size();

        drive_current = -ax_avg * g_DRIFT_GAIN;

        // manual-drive decay (unused here)
        float dec = g_OVERSHOOT_RATE * dt;
        if (fabs(drive_current) > dec)
            drive_current -= (drive_current>0?1:-1)*dec;
        else
            drive_current = -(drive_current>0?1:-1)*(dec - fabs(drive_current));

        // PID update
        float u_pid = pid.update(theta, theta_dot, dt, u_smoothed, -3e5f, 3e5f);
        float target = u_pid + drive_current;
        u_smoothed += (dt/g_SMOOTH_TAU)*(target - u_smoothed);
        return u_smoothed;
    }
};

// —————————————————————————————————————————————————
// Hardware & globals
// —————————————————————————————————————————————————
static const int STEPPER_INTERVAL_US = 100;
static const int dirPin_r       = 16, stepPin_r = 17;
static const int dirPin_l       = 4,  stepPin_l = 14;
static const int STEPPER_EN_PIN = 15, togglePin = 19;

static step      right_stepper(STEPPER_INTERVAL_US, stepPin_r, dirPin_r);
static step      left_stepper (STEPPER_INTERVAL_US, stepPin_l, dirPin_l);
static ESP32Timer stepTimer(3);
static Adafruit_MPU6050 mpu;
static RoverController ctrl;

unsigned long previous_time = 0;
float tilt_bias = 0.0f;  // measured pitch bias

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
    Serial.println("Balance Robot — tuned, faster reaction");

    // MPU init (SDA=21, SCL=22)
    Wire.begin(21, 22);
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // pins
    pinMode(dirPin_r, OUTPUT);
    pinMode(stepPin_r, OUTPUT);
    pinMode(dirPin_l, OUTPUT);
    pinMode(stepPin_l, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(togglePin, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW); // enable (active LOW)

    // set stepper accel
    constexpr float ACCEL_RAD = 60.0f;
    left_stepper .setAccelerationRad(ACCEL_RAD);
    right_stepper.setAccelerationRad(ACCEL_RAD);

    // start 100µs ISR
    if (!stepTimer.attachInterruptInterval(STEPPER_INTERVAL_US, onTimer)) {
        Serial.println("Failed to start ISR");
        while (1) delay(10);
    }

    // one-shot bias measurement
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
    // compute dt
    unsigned long now = micros();
    float dt = (now - previous_time) * 1e-6f;
    previous_time = now;
    if (dt < 1e-4f) dt = 1e-4f;

    // read IMU
    sensors_event_t ea, eg, et;
    mpu.getEvent(&ea, &eg, &et);
    float ax     = ea.acceleration.x / 16384.0f;
    float az     = ea.acceleration.z / 16384.0f;
    float gyro_y = eg.gyro.y;

    // raw pitch & corrected theta
    float raw     = atan2(-ax, az);               // rad
    float theta   = raw - tilt_bias;              // zero when level
    float theta_dot = (gyro_y/131.0f) * (PI/180.0f);

    // run controller
    float u = ctrl.step(theta, theta_dot, ax, dt);

    // drive steppers (flip left so both go same direction)
    right_stepper.setTargetSpeedRad( u);
    left_stepper .setTargetSpeedRad(-u);

    // debug
    Serial.print("dt: ");         Serial.print(dt,       4);
    Serial.print("  raw: ");      Serial.print(raw,      4);
    Serial.print("  theta: ");    Serial.print(theta,    4);
    Serial.print("  theta_dot: ");Serial.print(theta_dot,4);
    Serial.print("  u: ");        Serial.println(u,      4);

    delay(1);  // ~1 kHz loop
}
