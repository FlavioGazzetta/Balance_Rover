#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;

const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 20;
const int STEPPER_INTERVAL_US = 20;

const float kx = 20.0;
const float VREF = 4.096;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;

step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

bool TimerHandler(void* timerNo) {
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t TXByte0 = 0x06 | (channel >> 2);
  uint8_t TXByte1 = (channel & 0x03) << 6;
  digitalWrite(ADC_CS_PIN, LOW);
  SPI.transfer(TXByte0);
  uint8_t RXByte0 = SPI.transfer(TXByte1);
  uint8_t RXByte1 = SPI.transfer(0x00);
  digitalWrite(ADC_CS_PIN, HIGH);
  uint16_t result = ((RXByte0 & 0x0F) << 8) | RX1;
  return result;
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
  Serial.println("Initialised Interrupt for Stepper");

  step1.setAccelerationRad(15.0);
  step2.setAccelerationRad(15.0);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void loop() {
  static float Kp = 300.0;
  static float Ki = 40.0;
  static float Kd = 300.0;
  static float c = 0.96;
  static unsigned long previous_time = millis();
  static unsigned long printTimer = 0;
  static unsigned long loopTimer = 0;
  static float theta_n = 0.000;
  static float integral = 0.0;
  static float uoutput = 0.0;
  static float tilt_angle_x = 0.0;
  static float tilt_angle_y = 0.0;
  static float tilt_angle_z = 0.0;
  static float gyro_x = 0.0;
  static float gyro_y = 0.0;
  static float gyro_z = 0.0;
  static float error = 0.0;
  float speed1 = 0;
  float speed2 = 0;

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    tilt_angle_x = a.acceleration.x / 9.67;
    tilt_angle_y = a.acceleration.y / 9.67;
    tilt_angle_z = a.acceleration.z / 9.67;
    gyro_x = g.gyro.x;
    gyro_y = g.gyro.y;
    gyro_z = g.gyro.z;

    speed1 = step1.getSpeedRad();
    speed2 = step2.getSpeedRad();

    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    previous_time = current_time;

    theta_n = (1 - c) * tilt_angle_z + c * (gyro_y * dt + theta_n);

    float reference = -0.05;
    error = reference - theta_n;

    float derivative = -gyro_y;
    integral += error * dt;

    uoutput = Kp * error + Ki * integral + Kd * derivative;

    step1.setTargetSpeedRad(-uoutput);
    step2.setTargetSpeedRad(uoutput);
  }

  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("Speed1: ");
    Serial.print(speed1);
    Serial.print(" | Speed2: ");
    Serial.print(speed2);
    Serial.println();
  }
}
