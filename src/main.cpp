#include <esp32-hal-timer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "step.h"                  // your stepper driver
#include <TimerInterrupt_Generic.h>

// —————————————————————————————————————————————————
// Stepper & Timer constants
// —————————————————————————————————————————————————
const int STEPPER_INTERVAL_US = 100;
const int dirPin_r            = 16;
const int stepPin_r           = 17;
const int dirPin_l            = 4;
const int stepPin_l           = 14;
const int STEPPER_EN_PIN      = 15;
const int testpin             = 19;  // toggle pin in ISR

// Instantiate the steppers
static step right_stepper(STEPPER_INTERVAL_US, stepPin_r, dirPin_r);
static step left_stepper (STEPPER_INTERVAL_US, stepPin_l, dirPin_l);

// Timer object
static ESP32Timer ITimer(3);

// MPU object
Adafruit_MPU6050 mpu;

// PID Constants
float K_p = 20.0f;    // ↓ from 30.16  — reduces overall aggressiveness  
float K_i = 0.01f;    // ↓ from 0.03   — less wind-up, smoother steady-state  
float K_d = 7.5f; 

// PID State Variables
float previous_error_x = 0.0f;
float previous_error_y = 0.0f;
float integral_term_x = 0.0f;
float integral_term_y = 0.0f;
float pid_output_x    = 0.0f;
float pid_output_y    = 0.0f;

// Offset and desired angles
float bias_y        = 0.0f;
float desired_tilt  = -62.0f;
float desired_angle_x = 0.0f;
float desired_angle_y;

// Stepper control accel
float acc = 60.0f;  // rad/s²

// Last angles for filters
float last_x_angle = 0.0f;
float last_y_angle = 0.0f;
float last_z_angle = 0.0f;

// Calibration baselines
float base_x_accel                   = 0.0f;
float base_y_angle_from_accel_offset = 0.0f;
float base_z_accel                   = 0.0f;
float base_x_gyro                    = 0.0f;
float base_y_gyro                    = 0.0f;
float base_z_gyro                    = 0.0f;

// Timing
unsigned long previous_time = 0;
float dt = 0.0f;

// Kalman filter state for Y
float angle_estimate_y = 0.0f;
float P_y[2][2]        = {{1,0},{0,1}};
float Q_angle_y        = 0.001f;
float Q_bias_y         = 0.003f;
float R_measure_y      = 0.03f;

// Kalman filter update (unused in final code, but left in for reference)
float kalmanFilterY(float newAngle, float newRate, float dt) {
  float rate = newRate - bias_y;
  angle_estimate_y += dt * rate;

  P_y[0][0] += dt * (dt*P_y[1][1] - P_y[0][1] - P_y[1][0] + Q_angle_y);
  P_y[0][1] -= dt * P_y[1][1];
  P_y[1][0] -= dt * P_y[1][1];
  P_y[1][1] += Q_bias_y * dt;

  float S  = P_y[0][0] + R_measure_y;
  float K0 = P_y[0][0] / S;
  float K1 = P_y[1][0] / S;

  float y = newAngle - angle_estimate_y;
  angle_estimate_y += K0 * y;
  bias_y           += K1 * y;

  float P00 = P_y[0][0], P01 = P_y[0][1];
  P_y[0][0] -= K0 * P00;
  P_y[0][1] -= K0 * P01;
  P_y[1][0] -= K1 * P00;
  P_y[1][1] -= K1 * P01;

  return angle_estimate_y;
}

// ISR: update steppers + toggle test pin
bool TimerHandler(void*) {
  right_stepper.runStepper();
  left_stepper.runStepper();
  static bool toggle = false;
  digitalWrite(testpin, toggle);
  toggle = !toggle;
  return true;
}

// Sensor calibration
void calibrateSensors() {
  Serial.println("Calibrating...");
  const int N = 1000;
  float x_accel_sum = 0, z_accel_sum = 0;
  float y_angle_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  sensors_event_t ea, eg, et;

  for (int i = 0; i < N; i++) {
    mpu.getEvent(&ea, &eg, &et);
    x_accel_sum += ea.acceleration.x  / 16384.0f;
    z_accel_sum += ea.acceleration.z  / 16384.0f;
    y_angle_sum += atan2(-ea.acceleration.x,
                         sqrt(ea.acceleration.y*ea.acceleration.y +
                              ea.acceleration.z*ea.acceleration.z))
                   * 180.0f / M_PI;
    gx_sum += eg.gyro.x / 131.0f;
    gy_sum += eg.gyro.y / 131.0f;
    gz_sum += eg.gyro.z / 131.0f;

    if (i % 200 == 0) Serial.print('.');
    delay(3);
  }
  Serial.println("\nDone.");

  base_x_accel                   = x_accel_sum / N;
  base_y_angle_from_accel_offset = y_angle_sum / N;
  base_z_accel                   = z_accel_sum / N;
  base_x_gyro                    = gx_sum / N;
  base_y_gyro                    = gy_sum / N;
  base_z_gyro                    = gz_sum / N;

  Serial.printf("Offsets: x_accel=%.4f  y_ang=%.4f  x_gyro=%.4f  y_gyro=%.4f\n",
                base_x_accel,
                base_y_angle_from_accel_offset,
                base_x_gyro,
                base_y_gyro);
}

// Initialize steppers
void initializeStepperMotors() {
  left_stepper .setAccelerationRad(acc);
  right_stepper.setAccelerationRad(acc);
}

void setupMPU() {
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void setupPins() {
  pinMode(dirPin_r, OUTPUT);
  pinMode(dirPin_l, OUTPUT);
  pinMode(stepPin_r, OUTPUT);
  pinMode(stepPin_l, OUTPUT);
  pinMode(testpin, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);  // enable (active LOW)
}

void setupTimer() {
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  Serial.println("Stepper ISR started");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("PID Balance Test");

  desired_angle_y = bias_y + desired_tilt;

  setupMPU();
  setupPins();
  calibrateSensors();
  initializeStepperMotors();
  setupTimer();

  previous_time = millis();
  Serial.printf("Target angle: %.2f\n", desired_angle_y);
}

// Main control loop
void controlLoop() {
  unsigned long now = millis();
  dt = (now - previous_time) / 1000.0f;
  previous_time = now;
  if (dt <= 0) dt = 0.001f;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float accX = a.acceleration.x  / 16384.0f;
  float accY = a.acceleration.y  / 16384.0f;
  float accZ = a.acceleration.z  / 16384.0f;

  float x_angle_acc = atan2(accY,
                     sqrt(accX*accX + accZ*accZ))
                     * 180.0f / M_PI
                   - base_x_accel;

  float raw_y_acc = atan2(-accX, accZ) * 180.0f / M_PI;
  float y_angle_acc = raw_y_acc
                    - base_y_angle_from_accel_offset
                    + desired_tilt;

  float gyro_rate_y = (g.gyro.y / 131.0f) - base_y_gyro;
  float gyro_angle_x = ((g.gyro.x/131.0f) - base_x_gyro)*dt + last_x_angle;
  float gyro_angle_z = ((g.gyro.z/131.0f) - base_z_gyro)*dt + last_z_angle;

  float alpha = 0.96f;
  float final_pitch = alpha*(last_y_angle + gyro_rate_y*dt)
                    + (1.0f-alpha)*y_angle_acc;

  last_x_angle = gyro_angle_x;
  last_y_angle = final_pitch;
  last_z_angle = gyro_angle_z;

  float error_x = desired_angle_x - gyro_angle_x;
  float error_y = desired_angle_y - final_pitch;

  // PID Y
  float pY = K_p * error_y;
  integral_term_y += K_i * error_y * dt;
  float dY = K_d * (error_y - previous_error_y) / dt;
  pid_output_y = pY + integral_term_y + dY;

  // drive steppers
  float drive = pid_output_y * fabs(pid_output_y) * 0.5f;
  left_stepper .setTargetSpeedRad(drive * (M_PI/1600.0f));
  right_stepper.setTargetSpeedRad(-drive * (M_PI/1600.0f));

  previous_error_y = error_y;

  Serial.printf("Y_acc=%.2f  Pitch=%.2f  PID=%.2f\n",
                y_angle_acc, final_pitch, pid_output_y);
}

void loop() {
  controlLoop();
  delay(0.1); 
}
