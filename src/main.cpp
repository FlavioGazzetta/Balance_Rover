#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// pins
const int   STEPPER1_DIR_PIN     = 16;
const int   STEPPER1_STEP_PIN    = 17;
const int   STEPPER2_DIR_PIN     = 4;
const int   STEPPER2_STEP_PIN    = 14;
const int   STEPPER_EN_PIN       = 15;
const int   TOGGLE_PIN           = 32;

// timing
const int   PRINT_INTERVAL       = 500;    // ms
const int   INNER_INTERVAL       = 10;     // ms
const int   STEPPER_INTERVAL_US  = 10;     // µs
const unsigned long OUTER_INTERVAL = 200;  // ms

// your chosen reference tilt offset (radians):
const float REFERENCE_ANGLE = -0.078;

// ==== objects ====
ESP32Timer    ITimer(3);
Adafruit_MPU6050 mpu;
step          step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step          step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

bool TimerHandler(void*) {
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  step1.setAccelerationRad(15.0);
  step2.setAccelerationRad(15.0);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);
}

void loop() {
  unsigned long now = millis();

  // --- inner‐loop & PID state ---
  static float  Kp = 300.0, Ki = 40.0, Kd = 300.0, c = 0.96;
  static unsigned long previous_time = now, loopTimer = 0;
  static float  theta_n = 0.0, integral = 0.0, uoutput = 0.0;
  static float  tilt_angle_z = 0.0, gyro_y = 0.0, error = 0.0;

  // --- outer‐loop state ---
  static unsigned long outerTimer = 0;
  static float         positionEstimate = 0.0;
  static float         desiredPosition  = 0.0;  // joystick/pot/etc if you like
  static float         Kp_outer         = 0.3;
  static float         tiltSetpoint     = 0.0;
  float                reference        = REFERENCE_ANGLE;

  // --- diagnostics ---
  static unsigned long printTimer = 0;

  // 1) Inner balance loop (every INNER_INTERVAL ms)
  if (now - loopTimer >= INNER_INTERVAL) {
    loopTimer += INNER_INTERVAL;

    // integrate wheel‐speed → positionEstimate
    float spd1 = step1.getSpeedRad();
    float spd2 = step2.getSpeedRad();
    float avgSpd = 0.5 * (spd1 + spd2);
    positionEstimate += avgSpd * (INNER_INTERVAL / 1000.0);

    // read IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    tilt_angle_z = a.acceleration.z / 9.81;
    gyro_y       = g.gyro.pitch;

    // complementary filter
    float dt = (now - previous_time) / 1000.0;
    previous_time = now;
    theta_n = (1 - c)*tilt_angle_z + c*(theta_n + gyro_y*dt);

    // compute error (includes reference + outer‐loop setpoint)
    error = (reference + tiltSetpoint) - theta_n;
    integral += error * dt;
    float derivative = -gyro_y;
    uoutput = Kp*error + Ki*integral + Kd*derivative;

    // drive steppers
    step1.setTargetSpeedRad( uoutput);
    step2.setTargetSpeedRad( uoutput);
  }

  // 2) Outer position loop (every OUTER_INTERVAL ms)
  if (now - outerTimer >= OUTER_INTERVAL) {
    outerTimer += OUTER_INTERVAL;

    float posError = desiredPosition - positionEstimate;
    tiltSetpoint = Kp_outer * posError + reference;
    tiltSetpoint = constrain(tiltSetpoint,
                             reference - 0.1,
                             reference + 0.1);
  }

  // 3) Diagnostics print (every PRINT_INTERVAL ms)
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer += PRINT_INTERVAL;
    Serial.print("ref: ");        Serial.print(reference,  4);
    Serial.print(" | tilt_sp: "); Serial.print(tiltSetpoint,4);
    Serial.print(" | theta: ");   Serial.print(theta_n,     4);
    Serial.print(" | pos: ");     Serial.print(positionEstimate,4);
    Serial.print(" | spd1: ");    Serial.print(step1.getSpeedRad(),4);
    Serial.print(" | spd2: ");    Serial.println(step2.getSpeedRad(),4);
  }
}
