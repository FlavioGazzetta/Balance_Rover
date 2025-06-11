/********************************************************************
 * ESP32 Balance-Bot – combined firmware  (v3.2 • spin & auto-sync)
 *  • Inner‐PID + outer‐P controller (exact gains)
 *  • MPU6050 for tilt sensing
 *  • Wi-Fi UI: ↑ ↓ ← → buttons + numeric set-point
 *  • ↑/↓ tilt while held (“freeze on release”)
 *  • ←/→ spins in place
 *  • Straight‐line auto‐sync: wheels match speed when not spinning
 *  • Inner loop: 20 ms; Outer loop (position): 200 ms
 ********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <WiFi.h>
#include <WiFiUdp.h>          
#include <math.h>


const char* ssid = "cole";
const char* password = "abcabcabc";

WiFiUDP udp;
const int  UDP_PORT = 8888;

char udpBuf[32];              // temp buffer for the ASCII number
int  xCam = 0;                // latest horizontal offset

/* -------- camera intrinsics -------- */
const int   IMG_W      = 320;     // pixels
const float HFOV_deg   = 60.0;   // horizontal FOV of your lens

// focal length expressed in *pixels*:
const float FOCAL_PX = IMG_W / (2.0f * tan(HFOV_deg * 0.5f * M_PI / 180.0));
// → ≈ 277 px for 320-wide & 60° lens


/* ─────────────────────────── PIN MAP ─────────────────────────── */
const int STEPPER1_DIR_PIN   = 16;
const int STEPPER1_STEP_PIN  = 17;
const int STEPPER2_DIR_PIN   = 4;
const int STEPPER2_STEP_PIN  = 14;
const int STEPPER_EN_PIN     = 15;
const int TOGGLE_PIN         = 32;

/* ─────────────────────────── TIMING ──────────────────────────── */
const int PRINT_INTERVAL        = 500;      // ms
const int INNER_INTERVAL        = 20;       // ms
const int STEPPER_INTERVAL_US   = 20;       // µs
const unsigned long OUTER_INTERVAL = 100;   // ms

/* ───────────────────── PID & FILTER CONSTANTS ────────────────── */
const float Kp_inner        = 2000.0f;
const float Ki_inner        =    1.0f;
const float Kd_inner        =  200.0f;
const float c               =  0.96f;     // complementary‐filter coefficient
const float REFERENCE_ANGLE = -0.035f;    // rad

// Kd boost thresholds
const float ERROR_SMALL_THRESHOLD = 0.005f;   // rad
const float GYRO_SPIKE_THRESHOLD  = 0.2f;     // rad/s
const float Kd_BOOST_FACTOR       = 100.0f;



// outer (position) loop gain
const float ANGLE_CONSTRAINT = 0.025;

/* ───────────────────── MANUAL‐DRIVE CONSTANTS ────────────────── */
const float TILT_MANUAL = 1.0f;
const float TILT_ANGLE  = 0.015f;


const float ROT_OFFSET = 0.5f;  // rad  (≈8.4°)

static float          desiredHeading = 0.0f;   // set‐point
const  float          Kp_rot         = 0.1f;   // P‐gain

/* ─────────────────────────────────────────────────────────────────
   ───────────────────────────────────────────────────────────────── */

/* ─────────────────────────── FALL‐DETECTION ───────────────────── */
const float FALL_THRESHOLD = 0.5f;  // radians (~28.6°). Adjust as needed
volatile bool fallen = false;       // set true once robot “falls”
volatile float avgSpeed = 0;

const float POSERRLIMIT = 10;
const float POSERRSLOWLIMIT = 5;


/* ─────────────────────────── OBJECTS ─────────────────────────── */
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step             step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step             step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);


/* ──────────────────── STATE / SHARED FLAGS ───────────────────── */
volatile float  g_webDesired       = 0.0f;   // numeric set‐point from Wi-Fi

volatile bool   manualMode         = false;  // ↑ or ↓ held?
volatile float  manualTiltOffset   = 0.0f;   // ±TILT_ANGLE
volatile float  movementoffset     = 0.0f;   // unused elsewhere

volatile bool   wheelLeftMode      = false;  // ← held?
volatile bool   wheelRightMode     = false;  // → held?

volatile bool   freezePosition     = false;  // “hold here” flag
volatile float  freezePositionPos  = 0.0f;   // latched position (rad)

volatile float  g_positionEstimate = 0.0f;   // shared with /cmd (not used now)

volatile float  h_webDesired       = 0.0f;

const float H_DEAD = 0.9;

static unsigned long lastMicrosHeading = 0;


volatile float spinComp      = 0.0f;
volatile float prev_spinComp = 0.0f;
volatile float prev_spinErr  = 0.0f;
volatile float spinIntegral  = 0.0f;

// spin‐PID gains
const float tp = 2.0f;
const float td = 0.5f;
const float ti = 0.1f;

volatile float spinErr = 0;

volatile float spinDeriv = 0;

volatile float spinRate = 0;

volatile float dt = 0;


/* ---------- fallback-simulation control ---------- */
bool   useFake    = false;          // true ⇢ Wi-Fi failed, run simulator
float  areaCam    = 0;              // fake / real area value
unsigned long lastFakeMs = 0;       // timestamp of last fake sample
int    fakeStep   = 0;              // advances 1 step / sec



/* ───────────────────── STEPPER TIMER ISR ─────────────────────── */
bool IRAM_ATTR TimerHandler(void*) {
  static bool tog = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, tog);
  tog = !tog;
  return true;
}

/* ────────────────────────── SETUP ────────────────────────────── */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 Balance-Bot v3.2 (UDP follower) ===");

  unsigned long t0 = millis();
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 3000.0) {
    delay(400);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nConnected!  IP = ");
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);                 // start UDP only if Wi-Fi OK
  } else {
    Serial.println("\n*** Wi-Fi failed – entering FAKE SENSOR MODE ***");
    useFake = true;
  }


  /* Hardware init */
  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);


  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected! Halt.");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to attach stepper ISR");
    while (1) delay(10);
  }

  step1.setAccelerationRad(30.0f);
  step2.setAccelerationRad(30.0f);

}

/* ────────────────────────── MAIN LOOP ───────────────────────── */
void loop() {  
  /* ---------- sensor input: real UDP or fake ---------- */
unsigned long now = millis();

if (!useFake) {
  /* ------------ REAL UDP branch ------------ */
  int pktsz = udp.parsePacket();
  if (pktsz) {
    int n = udp.read(udpBuf, sizeof(udpBuf) - 1);
    if (n > 0) {
      udpBuf[n] = '\0';
      // Expected payload:  "x area"   e.g. "147 13250"
      xCam    = atoi(strtok(udpBuf, " "));
      areaCam = atof(strtok(nullptr, " "));
    }
    Serial.printf("UDP  x=%d  area=%.0f  (%d bytes)\n", xCam, areaCam, pktsz);
  }
} else {
  /* ------------ FAKE branch (1 sample / s) ------------ */
  /* ------------ FAKE branch (10 samples / s) ------------ */
  if (now - lastFakeMs >= 100) {          // 100 ms period  → 10 Hz update
      lastFakeMs = now;
      fakeStep  += 5;                    // 12° per tick → full cycle in 30 steps
      if (fakeStep >= 360) fakeStep -= 360;

      float phase = fakeStep * TWO_PI / 360.0f;   // 0 … 2π
      xCam    = 160 + (int)(160.0f * sinf(phase)); // 100 … 220
      areaCam = 10000.0f + 2500.0f * sinf(phase); // (unchanged example)

      Serial.printf("FAKE x=%d  area=%.0f\n", xCam, areaCam);
  }

}


  /* --------------- INNER LOOP (20 ms) ---------------- */
  static unsigned long prevTime = now, innerT = 0;
  static float theta = 0.0f, integral = 0.0f, uout = 0.0f;
  static float tilt_acc_z = 0.0f, gyro_y = 0.0f, err_inner = 0.0f;

  static float  gyro_z = 0;

  /* sync PI integrator */
  static float syncInt = 0.0f;

  /* --------------- OUTER LOOP (200 ms) --------------- */
  static unsigned long outerT = 0;
  static float desiredPos = 0.0f, tiltSP = 0.0f;

  /* --------------- Diagnostics (500 ms) --------------- */
  static unsigned long printT = 0;

  float ref = 0;

  float avgSpeed = 0;
  float SumSpeed = 0;
  float speedcount = 0;
  bool speedtoohigh = false;

  /* ~~~~~~~~~~~~~ INNER CONTROL LOOP ~~~~~~~~~~~~~ */
  if (now - innerT >= INNER_INTERVAL) {
    innerT += INNER_INTERVAL;

    static const float MAX_CORR = 0.6f;          // ≈ ±34 °
    int   xCamCentered = xCam - 160;   
    float deltaYaw     = ((xCamCentered/5.33333333333333334)*(PI/180));
    h_webDesired = deltaYaw;          // 

    // 1) Determine angle setpoint (reference) based on manual vs. auto
    if (manualMode) {
      ref = REFERENCE_ANGLE + manualTiltOffset;
    } else {
      ref = REFERENCE_ANGLE;
    }

    // 2) Read wheel speeds (rad/s)
    float sp1_meas = step1.getSpeedRad();
    float sp2_meas = step2.getSpeedRad();

    // 3) IMU reading & complementary filter
    sensors_event_t a, g, tmp;
    mpu.getEvent(&a, &g, &tmp);
    
    tilt_acc_z = atan2(a.acceleration.z, a.acceleration.x);
    gyro_y     = g.gyro.pitch;


    dt = (now - prevTime) / 1000.0f;  // in seconds
    prevTime = now;
    theta = (1.0f - c) * tilt_acc_z + c * (theta + gyro_y * dt);

    spinRate = g.gyro.roll + 0.0208; 

    // ─── FALL DETECTION & RECOVERY ─────────────────────────────────
    if (!fallen && fabsf(theta) > FALL_THRESHOLD) {
      fallen = true;
      Serial.println("Rover has fallen!");
    }
    else if (fallen && fabsf(theta) <= FALL_THRESHOLD) {
      fallen = false;
      Serial.println("Rover recovered, resuming operation");
      // (Optionally, reset integral if you want a fresh start)
      // integral = 0.0f;
    }

    err_inner = (ref + tiltSP) - theta;
    
    integral += err_inner * dt;
    float deriv = -gyro_y;
    float Kd_eff = Kd_inner;

    uout = Kp_inner * err_inner + Ki_inner * integral + Kd_eff * deriv;

    speedcount ++;

    SumSpeed += 0.5f * (sp1_meas + sp2_meas);

    avgSpeed = SumSpeed/speedcount;

    /* ----- DRIVE WHEELS BASED ON MODE (unless fallen) ----- */
    if (fallen) {
      // If fallen, force both motors to zero speed
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
    }
    else {  // Straight‐line auto‐sync / “balance” mode
        
        spinComp      = prev_spinComp + spinRate * dt;
        prev_spinComp = spinComp;

        // 2) PID on heading (turn_reference set elsewhere)
        spinErr   = h_webDesired - spinComp;
        spinDeriv = (spinErr - prev_spinErr) / dt;
        spinIntegral  += spinErr * dt;

        float Pto       = tp * spinErr;
        float Dto       = td * spinDeriv;
        float Ito       = ti * spinIntegral;
        float turnDrive = Pto + Dto + Ito;

        prev_spinErr = spinErr;

        // 3) Mix into your wheel commands alongside the balance‐drive (uout):
        step1.setTargetSpeedRad( uout  - turnDrive );
        step2.setTargetSpeedRad( uout  + turnDrive );

    }
  }

  /* ~~~~~~~~~~~~~ OUTER POSITION LOOP ~~~~~~~~~~~~~ */
  if (now - outerT >= OUTER_INTERVAL) {
    outerT += OUTER_INTERVAL;

    SumSpeed = 0;
    speedcount = 0;

    // 1) Read actual wheel positions in radians
    //    (average the two wheels → “robot center” rotation)
    float wheel1PosRad = step1.getPositionRad();
    float wheel2PosRad = step2.getPositionRad();
    float posEst = 0.5f * (wheel1PosRad + wheel2PosRad);

    // 2) If we just left manual mode, latch freezePositionPos:
    if (!manualMode && freezePosition) {
      // Once we drop out of manual, hold wherever we were
      freezePositionPos = posEst;
      // Keep freezePosition = true until user sets a new setpoint
    }

    // 3) Compute tiltSP based on manual vs. position control
    if (fallen) {
      // If fallen, lock tiltSP to zero so motors stay off
      tiltSP = 0;
    }
    else if (manualMode) {
      tiltSP = 0;  // override in manual mode
    }
    else {
      float desired = g_webDesired;
      float posErr  = desired - posEst;
      float absErr  = fabsf(posErr);
      float mult = 1;
      float Kp = 0;
      if(abs(posErr) > POSERRLIMIT){

        if(posErr < 0){

          Kp = (-0.05)/(2);

        }else{

          Kp = 0.05;

        }

        tiltSP = -constrain((Kp/abs(avgSpeed)), -ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT);


      }
      else if (absErr > POSERRSLOWLIMIT) {

        if(posErr < 0){

          Kp = (-0.025)/(2);

        }else{

          Kp = 0.025;

        }

        tiltSP = -constrain((Kp/abs(avgSpeed) * ((abs(posErr)-5)/10)), -ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT);

        }
      else{

        tiltSP = 0;

      }
      
    }

    avgSpeed = 0;
  }

  /* ~~~~~~~~~~~~~ DIAGNOSTICS PRINT & BUILD JSON (500 ms) ~~~~~~~~~~~~~ */
  if (now - printT >= PRINT_INTERVAL) {
    printT += PRINT_INTERVAL;

    // 1) Read wheel positions & compute headings
    float w1pos      = step1.getPositionRad();
    float w2pos      = step2.getPositionRad();
    float centerPos  = 0.5f * (w1pos + w2pos);

    // 2) Other telemetry
    float sp1        = step1.getSpeedRad();
    float sp2        = step2.getSpeedRad();
    float desiredOut = g_webDesired;
    int   Lm         = wheelLeftMode ? 1 : 0;
    int   Rm         = wheelRightMode ? 1 : 0;
    float diffSp     = sp1 - sp2;
    int   mMode      = manualMode ? 1 : 0;
    int   fFlag      = fallen ? 1 : 0;

    // 3) Print to Serial, including desiredHeading
    Serial.printf(
      "tiltSP %.3f | θ %.3f | heading %.3f | desiredHeading %.3f | "
      "posEst %.3f | w1pos %.3f | w2pos %.3f | sp1 %.3f | sp2 %.3f | "
      "desPos %.3f | L %d | R %d | diff %.3f | manual:%d | fallen:%d | "
      "avgSpeed:%.3f | Roll %.3f | pitch %.3f\n",
      tiltSP, theta, spinComp, h_webDesired,
      centerPos, w1pos, w2pos, sp1, sp2,
      desiredOut, Lm, Rm, diffSp, mMode, fFlag, avgSpeed, gyro_z, gyro_y
    );
  }

}
