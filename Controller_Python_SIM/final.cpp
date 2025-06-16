#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

static const int STEPPER1_DIR_PIN   = 16;
static const int STEPPER1_STEP_PIN  = 17;
static const int STEPPER2_DIR_PIN   = 4;
static const int STEPPER2_STEP_PIN  = 14;
static const int STEPPER_EN_PIN     = 15;

static const int ADC_CS_PIN         = 5;
static const int ADC_SCK_PIN        = 18;
static const int ADC_MISO_PIN       = 19;
static const int ADC_MOSI_PIN       = 23;

static const int TOGGLE_PIN         = 32;

static const int PRINT_INTERVAL     = 500;
static const int CTRL_INTERVAL      = 10;
static const int STEPPER_INTERVAL_US= 20;

extern "C" {
  void*    make_controller();
  float    update_controller(void*, float accel[3], float gyro[3], float slide_vel[1], float dt[1]);
  void     free_controller(void*);
  void     on_fwd_press(void*);
  void     on_fwd_release(void*);
  void     on_bwd_press(void*);
  void     on_bwd_release(void*);
  void     set_Kp_ang(float);
  void     set_Ki_ang(float);
  void     set_Kd_ang(float);
  void     set_drive_gain(float);
  void     set_drift_gain(float);
}

Adafruit_MPU6050   mpu;
ESP32Timer         stepperTimer(3);

step               step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step               step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

static void*      rover = nullptr;
volatile bool     tog = false;

bool IRAM_ATTR onStepperTimer(void*) {
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, tog);
  tog = !tog;
  return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t tx0 = 0x06 | (channel >> 2);
  uint8_t tx1 = (channel & 0x03) << 6;
  digitalWrite(ADC_CS_PIN, LOW);
  SPI.transfer(tx0);
  uint8_t hi = SPI.transfer(tx1);
  uint8_t lo = SPI.transfer(0);
  digitalWrite(ADC_CS_PIN, HIGH);
  return ((hi & 0x0F) << 8) | lo;
}

void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);
  step1.setAccelerationRad(10.0);
  step2.setAccelerationRad(10.0);

  if (!stepperTimer.attachInterruptInterval(STEPPER_INTERVAL_US, onStepperTimer)) {
    Serial.println("Failed to start stepper timer");
    while (1) delay(10);
  }

  rover = make_controller();
  Serial.println("Controller ready");
}

void loop() {
  static uint32_t nextCtrl = 0;
  static uint32_t nextPrint= 0;

  uint32_t now = millis();
  if (now >= nextCtrl) {
    nextCtrl = now + CTRL_INTERVAL;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accel[3]   = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
    float gyro[3]    = { g.gyro.x, g.gyro.y, g.gyro.z };
    float slide_vel[1] = { 0.0f };
    static uint32_t lastMicros = micros();
    float dt[1] = { (micros() - lastMicros) * 1e-6f };
    lastMicros = micros();

    float u = update_controller(rover, accel, gyro, slide_vel, dt);

    step1.setTargetSpeedRad(+u);
    step2.setTargetSpeedRad(-u);
  }

  if (now >= nextPrint) {
    nextPrint = now + PRINT_INTERVAL;
    float speed1 = step1.getSpeedRad();
    float volt   = (readADC(0) * 4.096f) / 4095.0f;
  }
}

void shutdownController() {
  if (rover) {
    free_controller(rover);
    rover = nullptr;
  }
}
