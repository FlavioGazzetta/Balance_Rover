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
static constexpr float PI_VALUE    = 3.14159265358979323846f;
static constexpr float DERIV_TAU   = 0.02f;

// adjust these gains to tune performance
static float Kp = 200.0f;
static float Ki =   10.0f;
static float Kd = 200.0f;
static float SMOOTH_TAU = 1.0f;

// -------------------------------------------------------------
// Wheel acceleration/deceleration constant
// -------------------------------------------------------------
static constexpr float MAX_WHEEL_ACCEL = 50.0f;  // rad/s^2
// -------------------------------------------------------------

// -------------------------------------------------------------
// Global PID state
// -------------------------------------------------------------
static float integralSum    = 0.0f;
static float prevError      = 0.0f;
static float derivFiltered  = 0.0f;
static float controlSmooth  = 0.0f;
static unsigned long prevLoopMicros = 0;
static float tiltBias       = 0.0f;

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
        tiltBias = atan2f(-ax0, az0);
        Serial.print("tilt bias (rad): ");
        Serial.println(tiltBias, 6);
    }

    // Attach stepper ISR
    if (!stepTimer.attachInterruptInterval(STEPPER_INTERVAL_US, onTimer)) {
        Serial.println("Failed to start stepper ISR");
        while (1) delay(10);
    }

    // Configure steppers
    step1.setAccelerationRad(MAX_WHEEL_ACCEL);
    step2.setAccelerationRad(MAX_WHEEL_ACCEL);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW);

    // Setup ADC SPI
    pinMode(ADC_CS_PIN, OUTPUT);
    digitalWrite(ADC_CS_PIN, HIGH);
    SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

    // Initialize loop timer
    prevLoopMicros = micros();
}

void loop() {
    static unsigned long nextPrint = 0;

    // Always keep these in scope for printing
    float tilt      = 0.0f;
    float tiltRate  = 0.0f;

    unsigned long nowMillis = millis();
    unsigned long nowMicros = micros();

    // Control loop
    if (nowMicros - prevLoopMicros >= LOOP_INTERVAL_MS * 1000UL) {
        float dt = (nowMicros - prevLoopMicros) * 1e-6f;
        prevLoopMicros = nowMicros;
        if (dt < 1e-4f) dt = 1e-4f;

        // Read IMU
        sensors_event_t ea, eg, et;
        mpu.getEvent(&ea, &eg, &et);
        float ax    = ea.acceleration.x / 16384.0f;
        float az    = ea.acceleration.z / 16384.0f;
        float gyroY = eg.gyro.y;

        // Compute tilt and tilt rate
        float rawTilt    = atan2f(-ax, az);
        tilt             = rawTilt - tiltBias;
        tiltRate         = (gyroY / 131.0f) * (PI_VALUE / 180.0f);

        // PID derivative
        float derivRaw   = (tilt - prevError) / dt;
        float alpha      = dt / (DERIV_TAU + dt);
        derivFiltered  += alpha * (derivRaw - derivFiltered);

        // PID output before smoothing
        float controlRaw = Kp * tilt + Ki * integralSum + Kd * derivFiltered;
        if (controlRaw > -1e6f && controlRaw < 1e6f) {
            integralSum += tilt * dt;
        }

        // Constrain and smooth
        float controlLimited = constrain(controlRaw, -3e5f, 3e5f);
        prevError            = tilt;
        controlSmooth       += (dt / SMOOTH_TAU) * (controlLimited - controlSmooth);

        // —— Predictive stop logic —————————————————————
        float wheelSpeed    = controlSmooth;
        float stopTime      = fabs(wheelSpeed) / MAX_WHEEL_ACCEL;
        float predictedTilt = tilt + tiltRate * stopTime;

        // If predicted tilt crosses zero (or is very small), stop wheels now
        if (predictedTilt * tilt <= 0.0f || fabs(predictedTilt) < 0.001f) {
            step1.setTargetSpeedRad(0.0f);
            step2.setTargetSpeedRad(0.0f);
        } else {
            step1.setTargetSpeedRad( controlSmooth);
            step2.setTargetSpeedRad(-controlSmooth);
        }
        // ——————————————————————————————————————————————

        // Diagnostic print
        if (nowMillis >= nextPrint) {
            nextPrint = nowMillis + PRINT_INTERVAL_MS;
            float voltage = (readADC(0) * 4.096f) / 4095.0f;
            Serial.print("control: ");       Serial.print(controlSmooth,4);
            Serial.print("  tilt (mrad): "); Serial.print(tilt*1000,2);
            Serial.print("  rate: ");         Serial.print(tiltRate,4);
            Serial.print("  predTilt: ");     Serial.println(predictedTilt,4);
        }
    }
}
