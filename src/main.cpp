#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// === pins ===
const int STEPPER1_DIR_PIN   = 16;
const int STEPPER1_STEP_PIN  = 17;
const int STEPPER2_DIR_PIN   = 4;
const int STEPPER2_STEP_PIN  = 14;
const int STEPPER_EN_PIN     = 15;
const int TOGGLE_PIN         = 32;

// === timing ===
const int PRINT_INTERVAL      = 500;   // ms
const int INNER_INTERVAL      = 20;    // ms
const int STEPPER_INTERVAL_US = 20;    // µs
const unsigned long OUTER_INTERVAL = 200; // ms

// === single-loop PID values ===
const float Kp_inner        = 1000.0;
const float Ki_inner        =    1.0;
const float Kd_inner        =  200.0;
const float c               =  0.96;   // complementary filter coefficient
const float REFERENCE_ANGLE = -0.045;  // radians

// === outer loop gain ===
const float Kp_outer = 0.3;

// === objects ===
ESP32Timer    ITimer(3);
Adafruit_MPU6050 mpu;
step          step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step          step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// interrupt: toggle pin + stepper ISR
bool TimerHandler(void*) {
  static bool tog = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, tog);
  tog = !tog;
  return true;
}

// read 12-bit SPI ADC, channel 0–7

void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  // MPU
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);


  // Stepper interrupt
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  // Stepper config
  step1.setAccelerationRad(15.0);
  step2.setAccelerationRad(15.0);
}

void loop() {
  unsigned long now = millis();

  // --- Inner‐loop state ---
  static unsigned long previous_time = now, innerTimer = 0;
  static float theta_n = 0, integral = 0, uoutput = 0;
  static float tilt_acc_z = 0, gyro_y = 0, error_inner = 0;

  // --- Outer‐loop state ---
  static unsigned long outerTimer = 0;
  static float positionEstimate = 0, desiredPosition = 0, tiltSetpoint = 0;

  // --- diagnostics ---
  static unsigned long printTimer = 0;

  // 1) INNER‐loop: every INNER_INTERVAL ms
  if (now - innerTimer >= INNER_INTERVAL) {
    innerTimer += INNER_INTERVAL;

    // integrate wheel‐speed → positionEstimate
    float sp1 = step1.getSpeedRad();
    float sp2 = step2.getSpeedRad();
    positionEstimate += 0.5*(sp1 + sp2)*(INNER_INTERVAL/1000.0);

    // read IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    tilt_acc_z = a.acceleration.z / 9.81;
    gyro_y     = g.gyro.y;

    // complementary filter
    float dt = (now - previous_time)/1000.0;
    previous_time = now;
    theta_n = (1-c)*tilt_acc_z + c*(theta_n + gyro_y*dt);

    // compute control
    if(abs(REFERENCE_ANGLE - theta_n) < 0.01){

      error_inner = (REFERENCE_ANGLE + tiltSetpoint) - theta_n;

    } else{

      error_inner = (REFERENCE_ANGLE) - theta_n;

    }
    integral   += error_inner * dt;
    float derivative = -gyro_y;
    uoutput = Kp_inner*error_inner
            + Ki_inner*integral
            + Kd_inner*derivative;

    // apply to steppers
    step1.setTargetSpeedRad(uoutput);
    step2.setTargetSpeedRad(uoutput);
  }

  // 2) OUTER‐loop: every OUTER_INTERVAL ms
  if (now - outerTimer >= OUTER_INTERVAL) {
    outerTimer += OUTER_INTERVAL;

    // read pot → desiredPosition
    desiredPosition = 0.0;

    // PI on position → tiltSetpoint
    float posErr = desiredPosition - positionEstimate;
    tiltSetpoint = Kp_outer * posErr;
    tiltSetpoint = constrain(
      tiltSetpoint,
      -0.086 - 0.1,
      -0.086 + 0.1
    );
  }

  // 3) Diagnostics: every PRINT_INTERVAL ms
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer += PRINT_INTERVAL;
    Serial.print("ref: ");        Serial.print(REFERENCE_ANGLE,4);
    Serial.print(" | til_sp: ");  Serial.print(tiltSetpoint,4);
    Serial.print(" | theta: ");   Serial.print(theta_n,   4);
    Serial.print(" | pos: ");     Serial.print(positionEstimate,4);
    Serial.print(" | sp1: ");     Serial.print(step1.getSpeedRad(),4);
    Serial.print(" | sp2: ");     Serial.println(step2.getSpeedRad(),4);
  }
}
