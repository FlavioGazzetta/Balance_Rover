#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "step.h"
#include <math.h>

// -------------------------------------------------------------
// Pin definitions
// -------------------------------------------------------------
const int STEPPER1_DIR_PIN    = 16;
const int STEPPER1_STEP_PIN   = 17;
const int STEPPER2_DIR_PIN    = 4;
const int STEPPER2_STEP_PIN   = 14;
const int STEPPER_EN_PIN      = 15;
const int TOGGLE_PIN          = 32;

const int ADC_CS_PIN          = 5;
const int ADC_SCK_PIN         = 18;
const int ADC_MISO_PIN        = 19;
const int ADC_MOSI_PIN        = 23;

// -------------------------------------------------------------
// Timing constants
// -------------------------------------------------------------
const int PRINT_INTERVAL_MS   = 500;
const int LOOP_INTERVAL_MS    = 10;
const int STEPPER_INTERVAL_US = 20;

// -------------------------------------------------------------
// PID tuning constants
// -------------------------------------------------------------
static constexpr float Pi        = 3.14159265358979323846f;
static constexpr float DERIV_TAU = 0.02f;

// adjust these gains to tune performance
static float Kp = 200.0f;
static float Ki =   5.0f;
static float Kd = 200.0f;
static float SMOOTH_TAU = 1.0f;

// -------------------------------------------------------------
// Global PID state
// -------------------------------------------------------------
static float integral    = 0.0f;
static float prev_error  = 0.0f;
static float d_filtered  = 0.0f;
static float u_smoothed  = 0.0f;
static unsigned long prev_loop_micros = 0;
static float tilt_bias   = 0.0f;

// -------------------------------------------------------------
// Hardware objects
// -------------------------------------------------------------
ESP32Timer stepTimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// -------------------------------------------------------------
// ISR: update steppers and toggle pin
// -------------------------------------------------------------
bool IRAM_ATTR onTimer(void*) {
    step1.runStepper();
    step2.runStepper();
    static bool t = false;
    digitalWrite(TOGGLE_PIN, t);
    t = !t;
    return true;
}

// -------------------------------------------------------------
// Read 12-bit ADC via SPI
// -------------------------------------------------------------
uint16_t readADC(uint8_t channel) {
    uint8_t tx0 = 0x06 | (channel >> 2);
    uint8_t tx1 = (channel & 0x03) << 6;
    digitalWrite(ADC_CS_PIN, LOW);
    SPI.transfer(tx0);
    uint8_t rx0 = SPI.transfer(tx1);
    uint8_t rx1 = SPI.transfer(0x00);
    digitalWrite(ADC_CS_PIN, HIGH);
    return ((rx0 & 0x0F) << 8) | rx1;
}

void setup() {
    Serial.begin(115200);
    pinMode(TOGGLE_PIN, OUTPUT);

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // Calibrate tilt bias
    delay(100);
    sensors_event_t ea, eg, et;
    mpu.getEvent(&ea, &eg, &et);
    {
        float ax0 = ea.acceleration.x / 16384.0f;
        float az0 = ea.acceleration.z / 16384.0f;
        tilt_bias = atan2f(-ax0, az0);
        Serial.print("tilt_bias (rad): ");
        Serial.println(tilt_bias, 6);
    }

    // Attach stepper ISR
    if (!stepTimer.attachInterruptInterval(STEPPER_INTERVAL_US, onTimer)) {
        Serial.println("Failed to start stepper ISR");
        while (1) delay(10);
    }

    // Configure steppers
    step1.setAccelerationRad(10.0f);
    step2.setAccelerationRad(10.0f);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);

    // Setup ADC SPI
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

    // Initialize loop timer
    prev_loop_micros = micros();
}

void loop() {
    static unsigned long nextPrint = 0;

    // Always keep theta/theta_dot in scope for printing
    float theta     = 0.0f;
    float theta_dot = 0.0f;

    unsigned long nowMillis = millis();
    unsigned long nowMicros = micros();

    // Control loop
    if (nowMicros - prev_loop_micros >= LOOP_INTERVAL_MS * 1000UL) {
        float dt = (nowMicros - prev_loop_micros) * 1e-6f;
        prev_loop_micros = nowMicros;
        if (dt < 1e-4f) dt = 1e-4f;

        // Read IMU
        sensors_event_t ea, eg, et;
        mpu.getEvent(&ea, &eg, &et);
        float ax     = ea.acceleration.x / 16384.0f;
        float az     = ea.acceleration.z / 16384.0f;
        float gyro_y = eg.gyro.y;

        // Compute tilt and rate
        float raw        = atan2f(-ax, az);
        theta            = raw - tilt_bias;
        theta_dot        = (gyro_y / 131.0f) * (PI / 180.0f);

        // PID calculations
        float d_raw    = (theta - prev_error) / dt;
        float alpha    = dt / (DERIV_TAU + dt);
        d_filtered    += alpha * (d_raw - d_filtered);

        float u_raw    = Kp * theta + Ki * integral + Kd * d_filtered;
        if (u_raw > -1e6f && u_raw < 1e6f)
            integral += theta * dt;

        float u_pid    = constrain(u_raw, -3e5f, 3e5f);
        prev_error     = theta;

        // Smooth output
        u_smoothed    += (dt / SMOOTH_TAU) * (u_pid - u_smoothed);

        // Drive steppers
        step1.setTargetSpeedRad( u_smoothed);
        step2.setTargetSpeedRad(-u_smoothed);

        // Diagnostic print inside control block
        if (nowMillis >= nextPrint) {
            nextPrint = nowMillis + PRINT_INTERVAL_MS;
            float v = (readADC(0) * 4.096f) / 4095.0f;
            Serial.print("u_sm: ");       Serial.print(u_smoothed,4);
            Serial.print("  theta(mrad): "); Serial.print(theta*1000,2);
            Serial.print("  V(A0): ");    Serial.println(v,3);
        }
    }
}
