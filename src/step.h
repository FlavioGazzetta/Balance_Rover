#pragma once
#include <Arduino.h>

class step {
public:
    const int MAX_SPEED = 20000;            // maximum motor speed (steps/s)
    const int MAX_SPEED_INTERVAL_US = 1000; // maximum interval for speed updates (us)
    const int SPEED_SCALE = 2000;           // integer speed units are steps per SPEED_SCALE seconds
    const int MICROSTEPS = 16;              // microsteps per full step
    const int STEPS = 200;                  // full steps per revolution
    const float STEP_ANGLE = (2.0 * PI) / (STEPS * MICROSTEPS);

    int32_t accel = 0;   // current acceleration (steps/s^2)
    int32_t tSpeed = 0;  // target speed (steps/(SPEED_SCALE*s))

    // constructor: call before use
    step(int interval_us, int8_t step_pin, int8_t dir_pin)
      : interval(interval_us), stepPin(step_pin), dirPin(dir_pin)
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        digitalWrite(stepPin, LOW);
    }

    // call every interval_us microseconds (from ISR)
    void runStepper() {
        speedTimer += interval;

        if (step_period != 0) {
            stepTimer += interval;
            if (stepTimer >= step_period) {
                digitalWrite(dirPin, speed > 0);
                digitalWrite(stepPin, HIGH);
                stepTimer -= step_period;
                delayMicroseconds(2);
                digitalWrite(stepPin, LOW);
                position += (speed > 0 ? 1 : -1);
            }
        }

        if (speedTimer >= MAX_SPEED_INTERVAL_US
         || (step_period == 0 && tSpeed != 0)
         || (step_period != 0 && speed == 0)) {
            updateSpeed();
        }
    }

    // set acceleration in rad/s^2 (not from ISR)
    void setAccelerationRad(float accelRad) {
        accel = static_cast<int32_t>(accelRad / STEP_ANGLE);
    }

    // set target speed in rad/s (not from ISR)
    void setTargetSpeedRad(float speedRad) {
        tSpeed = static_cast<int32_t>(speedRad * SPEED_SCALE / STEP_ANGLE);
    }

private:
    volatile int32_t stepTimer = 0;
    volatile int32_t speedTimer = 0;
    volatile int32_t step_period = 0;
    volatile int32_t position = 0;
    volatile int32_t speed = 0;
    int32_t interval;
    int8_t stepPin;
    int8_t dirPin;

    void updateSpeed() {
        int32_t a = accel < 0 ? -accel : accel;
        int32_t delta = a * speedTimer / (1000000 / SPEED_SCALE);

        if (speed < tSpeed) {
            speed += delta;
            if (speed > tSpeed) speed = tSpeed;
        } else if (speed > tSpeed) {
            speed -= delta;
            if (speed < tSpeed) speed = tSpeed;
        }

        // clamp to limits
        if (speed > MAX_SPEED * SPEED_SCALE)  speed = MAX_SPEED * SPEED_SCALE;
        if (speed < -MAX_SPEED * SPEED_SCALE) speed = -MAX_SPEED * SPEED_SCALE;

        speedTimer = 0;
        if (speed == 0) {
            step_period = 0;
        } else if (speed > 0) {
            step_period = (1000000LL * SPEED_SCALE) / speed;
        } else {
            step_period = (1000000LL * SPEED_SCALE) / -speed;
        }
    }
};