#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

#include <WiFi.h>      // for Wi-Fi
#include <WebServer.h> // built-in Arduino web server

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

// --- thresholds for dynamic Kd boost ---
const float ERROR_SMALL_THRESHOLD   = 0.005;   // radians: “error is very small”
const float GYRO_SPIKE_THRESHOLD    = 0.2;     // rad/s: “gyro‐y spike is large”
const float Kd_BOOST_FACTOR         = 100.0;    // multiply Kd by this when both conditions met

// === outer loop gain ===
const float Kp_outer = 0.3;

// === objects for balancing ===
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step             step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step             step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// —————————————————————————————————————————————————————————————————
//  ▼ WEB SERVER + global for “web setpoint” ▼
// —————————————————————————————————————————————————————————————————
WebServer server(80);          // runs on port 80

// This global will hold the latest float entered via the web form:
volatile float g_webDesired = 0.0f;

// Serve the root page (HTML form)
void handleRoot() {
  const char* html =
    "<!DOCTYPE html>"
    "<html>"
    "<head><meta name='viewport' content='width=device-width, initial-scale=1.0'></head>"
    "<body>"
      "<h2>ESP32 BalanceBot</h2>"
      "<form action='/set' method='GET'>"
        "Set desiredPosition:<br>"
        "<input type='number' step='0.01' name='val' value='0.00'><br><br>"
        "<input type='submit' value='Submit'>"
      "</form>"
    "</body>"
    "</html>";
  server.send(200, "text/html", html);
}

// Handle “/set?val=2.50” (or any float)
void handleSet() {
  if (server.hasArg("val")) {
    String s = server.arg("val");
    float f = s.toFloat();
    g_webDesired = f;
    Serial.printf("[WEB] handleSet: g_webDesired = %.4f\n", g_webDesired);
    // Respond with a confirmation
    String resp = "Received: " + String(f, 4) + "<br><a href='/'>Back</a>";
    server.send(200, "text/html", resp);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}
// —————————————————————————————————————————————————————————————————
//  ▲ end of web server code ▲
// —————————————————————————————————————————————————————————————————

bool TimerHandler(void*) {
  static bool tog = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, tog);
  tog = !tog;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 BalanceBot (Wi-Fi + Web UI) ===");

  // —————————————————————————————————
  //  ▼ Bring up Wi-Fi as an Access Point ▼
  // —————————————————————————————————
  WiFi.softAP("ESP32_BalanceBot", "12345678");
    // SSID: “ESP32_BalanceBot”
    // Password: “12345678”

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);     // e.g. “192.168.4.1”

  // Set up URL handlers:
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
  Serial.println("Web server started. Visit http://192.168.4.1");

  // ——————————————————————————————————————————————————————
  //  ▼ Your original setup (MPU6050, TimerInterrupt, steppers) ▼
  //  (unchanged from your code)
  // ——————————————————————————————————————————————————————
  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  step1.setAccelerationRad(15.0);
  step2.setAccelerationRad(15.0);
}

void loop() {
  unsigned long now = millis();

  // — Serve any incoming HTTP requests (non-blocking) —
  server.handleClient();

  // --- Inner‐loop state ---
  static unsigned long previous_time = now, innerTimer = 0;
  static float theta_n = 0, integral = 0, uoutput = 0;
  static float tilt_acc_z = 0, gyro_y = 0, error_inner = 0;

  // --- Outer‐loop state ---
  static unsigned long outerTimer = 0;
  static float positionEstimate = 0;
  static float desiredPosition = 0;
  static float tiltSetpoint = 0;

  // --- diagnostics ---
  static unsigned long printTimer = 0;

  // 1) INNER‐loop: every INNER_INTERVAL ms
  if (now - innerTimer >= INNER_INTERVAL) {
    innerTimer += INNER_INTERVAL;

    // integrate wheel‐speed → positionEstimate
    float sp1 = step1.getSpeedRad();
    float sp2 = step2.getSpeedRad();
    positionEstimate += 0.5 * (sp1 + sp2) * (INNER_INTERVAL / 1000.0);

    // read IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    tilt_acc_z = a.acceleration.z / 9.81;
    gyro_y     = g.gyro.y;

    // complementary filter for tilt angle
    float dt = (now - previous_time) / 1000.0;
    previous_time = now;
    theta_n = (1 - c) * tilt_acc_z + c * (theta_n + gyro_y * dt);

    // — compute inner-loop error —
    if (fabsf(REFERENCE_ANGLE - theta_n) < 0.01) {
      error_inner = (REFERENCE_ANGLE + tiltSetpoint) - theta_n;
    } else {
      error_inner = (REFERENCE_ANGLE) - theta_n;
    }

    // integrate the error
    integral += error_inner * dt;

    // compute “raw” derivative term
    float derivative = -gyro_y;

    // dynamically adjust Kd if error is tiny AND gyro-y just spiked
    float Kd_effective = Kd_inner;  // default
    if ((fabsf(error_inner) < ERROR_SMALL_THRESHOLD)
        && (fabsf(gyro_y)     > GYRO_SPIKE_THRESHOLD)) {
      Kd_effective = Kd_inner * Kd_BOOST_FACTOR;
    }

    // final PID output using the possibly-boosted Kd
    uoutput = Kp_inner * error_inner
            + Ki_inner * integral
            + Kd_effective * derivative;

    // apply to steppers
    step1.setTargetSpeedRad(uoutput);
    step2.setTargetSpeedRad(uoutput);
  }

  // 2) OUTER‐loop: every OUTER_INTERVAL ms
  if (now - outerTimer >= OUTER_INTERVAL) {
    outerTimer += OUTER_INTERVAL;

    // read pot → desiredPosition (was stubbed as 0.0)
    // now override it from our web page float:
    desiredPosition = g_webDesired;

    // PI on position → tiltSetpoint
    float posErr = desiredPosition - positionEstimate;
    tiltSetpoint = Kp_outer * posErr;
    // Temporarily allow a bigger tilt so we actually see movement:
    tiltSetpoint = constrain(tiltSetpoint, -0.5, +0.5);
  }

  // 3) Diagnostics: every PRINT_INTERVAL ms
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer += PRINT_INTERVAL;
    Serial.print("ref: ");        Serial.print(REFERENCE_ANGLE, 4);
    Serial.print(" | til_sp: ");  Serial.print(tiltSetpoint,    4);
    Serial.print(" | theta: ");   Serial.print(theta_n,        4);
    Serial.print(" | pos: ");     Serial.print(positionEstimate,4);
    Serial.print(" | sp1: ");     Serial.print(step1.getSpeedRad(),4);
    Serial.print(" | sp2: ");     Serial.print(step2.getSpeedRad(),4);
    Serial.print(" | desPos: ");  Serial.print(desiredPosition, 4);
    Serial.println();
  }
}
