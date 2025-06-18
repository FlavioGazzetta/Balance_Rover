#include <esp32-hal-timer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
// #include <ContinuousStepper.h> // Will be removed
#include <step.h> // Added - Ensure step.h is in a library path (e.g., lib/step/step.h)
#include <TimerInterrupt_Generic.h> // Added

// Stepper and Timer constants from your example
// const int STEPPER_INTERVAL_US = 100; // Increased ISR interval to 100us. Try 250us or 500us if 100us still crashes.
const int STEPPER_INTERVAL_US = 100; // More reasonable ISR interval (0.25ms)

// Stepper Control Pins (defined earlier to resolve linter errors)
const int dirPin_r = 16; 
const int stepPin_r = 17;
const int dirPin_l = 4; 
const int stepPin_l = 14;
const int STEPPER_EN_PIN = 15;
const int testpin = 19; // For interrupt handler toggle, was 32 in example, using existing one for now

// Instantiate the steppers using step.h class & map to existing pin names
static step right_stepper(STEPPER_INTERVAL_US, stepPin_r, dirPin_r); // Corresponds to example's step1
static step left_stepper(STEPPER_INTERVAL_US, stepPin_l, dirPin_l);   // Corresponds to example's step2

// Instantiate the MPU
Adafruit_MPU6050 mpu;

// Timer object from your example
ESP32Timer ITimer(3); // Using timer 3 as in example
// hw_timer_t *timer = NULL; // Removed old timer

// Offset - bias_y is used by your Kalman filter
float bias_y = 0; //  Kalman filter uses it.
float desired_tilt = 0; // Added -3 degree tilt bias
float desired_angle_y; 

// LQR Control Variables
float k_lqr[4] = {18.8f, -14346.3f, 40.1f, -75.0f}; // LQR gain matrix
float lqr_pitch = 0.0f;          // Current pitch for LQR (degrees), from angle_y
float lqr_previous_pitch = 0.0f; // Previous pitch for LQR (degrees)
float lqr_pitch_dot = 0.0f;      // Current pitch rate for LQR (degrees/second)
float acc = 100.0f;
// LQR Setpoints
float lqr_pitch_set_point = 0.0f;    // Adjusted with -3 degree offset
float lqr_encoder_set_point = 0.0f;   // (Not used in simplified LQR)
float lqr_velocity_set_point = 0.0f;  // (Not used in simplified LQR)
float lqr_pitch_dot_set_point = 0.0f; // Desired pitch velocity (0 for balancing)

// LQR Motor control outputs
float u_lqr[2]; 
float v_lqr[2]; 
float lqr_scaling_factor = 0.35f; // Reduced from 0.45f for more balanced response
float max_stepper_speed = 8000.0f; // Reduced from 10000.0f for more controlled movement

// Angle Variables for complementary filter state (restored from original)
float last_x_angle = 0.0f, last_y_angle = 0.0f, last_z_angle = 0.0f;

// Sensor Calibration Baselines 
float base_x_accel = 0.0f; // Keep for x_angle_acc calculation as it was
float base_y_angle_from_accel_offset = 0.0f; // New: To store Y-axis angle offset from accelerometer in degrees
float base_z_accel = 0.0f; // Keep for x_angle_acc calculation as it was
float base_x_gyro = 0.0f; 
float base_y_gyro = 0.0f; 
float base_z_gyro = 0.0f;

// Timing
unsigned long previous_time = 0;
float dt = 0.0;

// ==== Kalman filter for Y angle ====
float angle_estimate_y = 0.0;
float P_y[2][2] = {{1, 0}, {0, 1}};
float Q_angle_y = 0.001;
float Q_bias_y = 0.003;
float R_measure_y = 0.03;

float kalmanFilterY(float newAngle, float newRate, float dt) {
  float rate = newRate - bias_y;
  angle_estimate_y += dt * rate;

  P_y[0][0] += dt * (dt * P_y[1][1] - P_y[0][1] - P_y[1][0] + Q_angle_y);
  P_y[0][1] -= dt * P_y[1][1];
  P_y[1][0] -= dt * P_y[1][1];
  P_y[1][1] += Q_bias_y * dt;

  float S = P_y[0][0] + R_measure_y;
  float K_0 = P_y[0][0] / S;
  float K_1 = P_y[1][0] / S;

  float y = newAngle - angle_estimate_y;
  angle_estimate_y += K_0 * y;
  bias_y += K_1 * y;

  float P00_temp = P_y[0][0];
  float P01_temp = P_y[0][1];

  P_y[0][0] -= K_0 * P00_temp;
  P_y[0][1] -= K_0 * P01_temp;
  P_y[1][0] -= K_1 * P00_temp;
  P_y[1][1] -= K_1 * P01_temp;

  return angle_estimate_y;
}

// Interrupt Service Routine for motor update (from your example)
// Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo) // Changed from IRAM_ATTR interruptHandler()
{
  static bool toggle = false;

  //Update the stepper motors
  right_stepper.runStepper(); // Temporarily disabled for WDT debugging
  left_stepper.runStepper();  // Temporarily disabled for WDT debugging

  //Indicate that the ISR is running
  digitalWrite(testpin,toggle);  
  toggle = !toggle;
	return true;
}

// calibrateSensors function 
void calibrateSensors() {
  Serial.println("Calibrating...");
  const int num_readings = 1000;
  float x_accel_sum = 0, z_accel_sum = 0; // For original x_angle_acc calculation
  float y_angle_sum_from_accel = 0; // For new y_angle_acc offset
  float x_gyro_sum = 0, y_gyro_sum = 0, z_gyro_sum = 0;

  sensors_event_t ea, eg, etemp;

  for (int i = 0; i < num_readings; i++) {
    mpu.getEvent(&ea, &eg, &etemp);
    // Sum for original base_x_accel and base_z_accel (g-force component offsets)
    x_accel_sum += ea.acceleration.x / 16384.0f;
    // y_accel_sum is not directly used for an offset in this new approach for Y-axis
    z_accel_sum += ea.acceleration.z / 16384.0f;

    // Sum for new base_y_angle_from_accel_offset (angle offset in degrees)
    // Uses raw accel values, not scaled ones, for atan2, then converts to degrees
    y_angle_sum_from_accel += atan2(-1.0f * ea.acceleration.x, sqrt(pow(ea.acceleration.y, 2) + pow(ea.acceleration.z, 2))) * 180.0f / PI;

    // Sum for gyro rate offsets (scaled rates)
    x_gyro_sum += eg.gyro.x / 131.0f;
    y_gyro_sum += eg.gyro.y / 131.0f;
    z_gyro_sum += eg.gyro.z / 131.0f;
    if (i % 200 == 0) { // Progress indicator
        Serial.print(".");
    }
    delay(3); 
  }
  Serial.println("\nCalculating averages...");

  base_x_accel = x_accel_sum / num_readings; // Retained for x_angle_acc original logic
  base_y_angle_from_accel_offset = y_angle_sum_from_accel / num_readings; // New Y angle offset
  base_z_accel = z_accel_sum / num_readings; // Retained for x_angle_acc original logic
  
  base_x_gyro = x_gyro_sum / num_readings;
  base_y_gyro = y_gyro_sum / num_readings;
  base_z_gyro = z_gyro_sum / num_readings;

  Serial.println("Calibration Completed.");
  Serial.println("-----------------------------------------------------");
  Serial.print("Base X Accel (g-component): "); Serial.println(base_x_accel, 4);
  Serial.print("Base Y Angle Offset from Accel (degrees): "); Serial.println(base_y_angle_from_accel_offset, 4);
  Serial.print("Base Z Accel (g-component): "); Serial.println(base_z_accel, 4);
  Serial.println("-----------------------------------------------------");
  Serial.print("Base X Gyro (scaled rate): "); Serial.println(base_x_gyro, 4);
  Serial.print("Base Y Gyro (scaled rate): "); Serial.println(base_y_gyro, 4);
  Serial.print("Base Z Gyro (scaled rate): "); Serial.println(base_z_gyro, 4);
  Serial.println("-----------------------------------------------------");
}

void initializeStepperMotors() {
  // left_stepper.begin(14, 4); // Constructor in step.h handles pins
  // right_stepper.begin(17, 16); // Constructor in step.h handles pins
  
  // Set motor acceleration values from example (10.0 rad/s^2)
  left_stepper.setAccelerationRad(acc); 
  right_stepper.setAccelerationRad(acc);
  // Old acceleration values were: left_stepper.setAcceleration(5600); right_stepper.setAcceleration(5600);
}

void setupMPU() {
  Wire.begin(20, 21);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
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
  pinMode(STEPPER_EN_PIN,OUTPUT); // Added from example
  digitalWrite(STEPPER_EN_PIN, false); // Set to false to ENABLE stepper drivers (assuming active LOW)
}

void setupTimer() {
  // timer = timerBegin(0, 20, true); // Removed old timer setup
  // timerAttachInterrupt(timer, &TimerHandler, true);
  // timerAlarmWrite(timer, 50, true);
  // timerAlarmEnable(timer);
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt from ITimer");
    while (1) delay(10);
  }
  Serial.println("Initialised Interrupt for Stepper using ITimer.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("LQR Test based on Working PID Code!");

  desired_angle_y = bias_y + desired_tilt; // Initialize desired_angle_y for Kalman init

  setupMPU();
  setupPins();
  calibrateSensors();
  initializeStepperMotors();
  setupTimer();

  angle_estimate_y = desired_angle_y;  // Initialize Kalman filter state  ->  might need review if Kalman is not used by LQR
  
  // Initialize LQR and complementary filter state variables
  last_x_angle = 0.0f;
  last_y_angle = 0.0f;
  last_z_angle = 0.0f; 

  lqr_pitch = lqr_pitch_set_point; // Initialize LQR pitch to its setpoint
  lqr_previous_pitch = lqr_pitch_set_point;
  
  previous_time = millis(); // Initialize previous_time for main loop dt calculation
  Serial.print("LQR Pitch Setpoint Initialized to: "); Serial.println(lqr_pitch, 4);
}

// LQR Pitch Derivative Calculation
void calculate_lqr_pitch_dot() {
    if (dt <= 0) dt = 0.001; // Safety for dt, should be updated in main loop
    lqr_pitch_dot = (lqr_pitch - lqr_previous_pitch) / dt;
    lqr_previous_pitch = lqr_pitch;
}

// LQR Control Application
void apply_lqr_control() {
    // Simplified LQR law with symmetric response adjustment
    float pitch_error = lqr_pitch_set_point - lqr_pitch;
    float pitch_dot_error = lqr_pitch_dot_set_point - lqr_pitch_dot;
    
    // Apply symmetric gain scaling based on direction
    float direction_scale = (pitch_error < 0) ? 0.7f : 1.0f;  // Reduce forward gain to match backward
    
    u_lqr[0] = direction_scale * (k_lqr[1] * pitch_error + k_lqr[3] * pitch_dot_error);
    u_lqr[1] = 0; // No yaw control

    v_lqr[0] = 0.5f * u_lqr[0]; // Right motor 
    v_lqr[1] = 0.5f * u_lqr[0]; // Left motor

    float lqr_output_scaled = u_lqr[0] * lqr_scaling_factor;
    lqr_output_scaled = constrain(lqr_output_scaled, -max_stepper_speed, max_stepper_speed);

    // Convert lqr_output_scaled to rad/s for step.h setTargetSpeedRad
    float target_speed_rad_s_left = -lqr_output_scaled * (PI / 1600.0f); 
    float target_speed_rad_s_right = lqr_output_scaled * (PI / 1600.0f);

    left_stepper.setTargetSpeedRad(-target_speed_rad_s_left);
    right_stepper.setTargetSpeedRad(-target_speed_rad_s_right); 
}

// void controlLoop() { 
void controlLoop_LQR() {
  unsigned long current_time = millis();
  dt = (current_time - previous_time) / 1000.0;
  previous_time = current_time;
  if (dt <= 0) dt = 0.001; // Safety

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Accelerometer readings (scaled, but not yet offset)
  float accX_raw_scaled = a.acceleration.x / 16384.0f;
  float accY_raw_scaled = a.acceleration.y / 16384.0f;
  float accZ_raw_scaled = a.acceleration.z / 16384.0f;

  // Calculate accelerometer-derived angles with improved backward tilt detection
  float x_angle_acc = atan2(accY_raw_scaled, sqrt(pow(accX_raw_scaled, 2) + pow(accZ_raw_scaled, 2))) * 180.0f / PI - base_x_accel;
  
  // Modified Y-angle calculation for better backward tilt detection
  float raw_y_angle_from_accel = atan2(-1.0f * accX_raw_scaled, accZ_raw_scaled) * 180.0f / PI;
  float y_angle_acc = raw_y_angle_from_accel - base_y_angle_from_accel_offset + desired_tilt; // Added desired_tilt offset
  
  // Gyro readings (rate, scaled and offset by calibration values)
  float gyro_rate_y_for_kalman = (g.gyro.y / 131.0f) - base_y_gyro;

  // Integrate gyro rates with improved numerical integration
  float gyro_angle_x = ((g.gyro.x / 131.0f) - base_x_gyro) * dt + last_x_angle;
  float gyro_angle_z = ((g.gyro.z / 131.0f) - base_z_gyro) * dt + last_z_angle;
  
  // Kalman filter for Y angle (pitch)
  float angle_y_kalman_output = kalmanFilterY(y_angle_acc, gyro_rate_y_for_kalman, dt);

  // Improved complementary filter with adaptive gain
  float alpha = 0.96f; // Increased trust in gyro for smoother response
  float alpha_threshold = 0.2f; // Threshold for acceleration-based angle change
  
  // Reduce alpha when acceleration is high (quick movements)
  if (fabs(y_angle_acc - last_y_angle) > alpha_threshold) {
    alpha = 0.7f;
  }
  
  float final_estimated_pitch = alpha * (last_y_angle + gyro_rate_y_for_kalman * dt) + 
                               (1.0f - alpha) * y_angle_acc;
  
  // Update LQR pitch state
  lqr_pitch = final_estimated_pitch;

  // Update last angles for the next iteration
  last_x_angle = gyro_angle_x;
  last_y_angle = lqr_pitch;
  last_z_angle = gyro_angle_z;

  // Calculate LQR pitch derivative with improved filtering
  calculate_lqr_pitch_dot();

  // Apply LQR control
  apply_lqr_control();

  // Debug output
  Serial.print("Y_ACC: "); Serial.print(y_angle_acc, 2);
  Serial.print(" LQR_Pitch: "); Serial.print(lqr_pitch, 2);
  Serial.print(" LQR_Dot: "); Serial.print(lqr_pitch_dot, 2);
  Serial.print(" LQR_U0: "); Serial.println(u_lqr[0], 2);
}

void loop() {
  controlLoop_LQR(); 
  // delay(1); 
}
