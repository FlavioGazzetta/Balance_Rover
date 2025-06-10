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
#include <WebServer.h>

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
const float REFERENCE_ANGLE = -0.04f;    // rad

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

/* ───────────────────────── WEB SERVER ────────────────────────── */
WebServer server(80);

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


/* ─────────────────────── TELEMETRY JSON ─────────────────────── */
// We will rebuild this JSON each time in diagnostics:
String statusJSON = "{}";


/* ─────────────────────── HTML HOMEPAGE ───────────────────────── */
const char HOMEPAGE[] PROGMEM = R"====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta
  name="viewport"
  content="width=device-width,initial-scale=1,maximum-scale=1,viewport-fit=cover"
    />
<title>Balance-Bot Controller</title>

<!-- ——————  minimal styles for a clean table —————— -->
<style>
  body {
    margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: #f4f5f7;
    display: flex;
    justify-content: center;
    padding: 20px;
  }
  .card {
    background: #ffffff;
    border-radius: 12px;
    box-shadow: 0 4px 12px rgba(0,0,0,0.1);
    max-width: 500px;
    width: 100%;
    padding: 20px;
  }
  h1 {
    margin: 0;
    font-size: 1.4rem;
    text-align: center;
    color: #333;
  }
  .pad {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    grid-template-rows: repeat(3, 1fr);
    gap: 12px;
    justify-items: center;
    align-items: center;
    margin: 20px 0;
  }
  .pad button {
    width: 60px;
    height: 60px;
    border: none;
    font-size: 1.2rem;
    font-weight: 600;
    border-radius: 50%;
    background: #e0e2e5;
    color: #333;
    cursor: pointer;
    transition: background .1s, transform .1s;
    user-select: none;          /* disable text selection */
    -webkit-user-select: none;  /* Safari/Chrome */
    -moz-user-select: none;     /* Firefox */
  }
  .pad button:active,
  .pad button.pressed {
    background: #3f51b5;
    color: #fff;
    transform: scale(0.96);
  }
  form {
    display: flex;
    flex-direction: column;
    gap: 10px;
    text-align: center;
    margin-bottom: 20px;
  }
  input[type=number] {
    padding: 8px;
    border: 1px solid #ccc;
    border-radius: 8px;
    width: 100px;
    margin: 0 auto;
    font-size: 1rem;
    text-align: center;
  }
  input[type=submit] {
    padding: 10px 0;
    border: none;
    border-radius: 8px;
    background: #3f51b5;
    color: #fff;
    font-size: 1rem;
    font-weight: 600;
    cursor: pointer;
    transition: background .1s;
  }
  input[type=submit]:hover {
    background: #303f9f;
  }
  /* Telemetry table */
  table {
    width: 100%;
    border-collapse: collapse;
    margin-top: 10px;
  }
  th, td {
    padding: 8px 6px;
    text-align: left;
    border-bottom: 1px solid #ddd;
  }
  th {
    background: #f1f3f7;
    color: #444;
    font-weight: 600;
  }
  td.value {
    font-weight: 500;
    color: #222;
  }
</style>

<!-- ——————  logic for binding buttons + fetching telemetry —————— -->
<script>
  function send(cmd) {
    fetch("/cmd?act=" + cmd);
  }

  function bind(id, start, stop) {
    const el = document.getElementById(id);
    let rpt = null;
    const down = (e) => {
      e.preventDefault();
      el.classList.add("pressed");
      send(start);
      rpt = setInterval(() => send(start), 200);
    };
    const up = (e) => {
      e.preventDefault();
      el.classList.remove("pressed");
      clearInterval(rpt);
      rpt = null;
      send(stop);
    };
    el.addEventListener("pointerdown", down, { passive: false });
    ["pointerup", "pointercancel", "pointerleave"].forEach((ev) =>
      el.addEventListener(ev, up, { passive: false })
    );
  }

  window.addEventListener("load", () => {
    // 1) Bind the arrow‐buttons
    bind("up", "up_start", "up_stop");
    bind("down", "down_start", "down_stop");
    bind("left", "left_start", "left_stop");
    bind("right", "right_start", "right_stop");

    // 2) Every 500 ms, fetch /status and update the telemetry table
    setInterval(async () => {
      try {
        const resp = await fetch("/status");
        if (!resp.ok) return;
        const data = await resp.json();
        document.getElementById("tiltSP").innerText   = data.tiltSP.toFixed(3);
        document.getElementById("theta").innerText    = data.theta.toFixed(3);
        document.getElementById("posEst").innerText   = data.posEst.toFixed(3);
        document.getElementById("w1pos").innerText    = data.w1pos.toFixed(3);
        document.getElementById("w2pos").innerText    = data.w2pos.toFixed(3);
        document.getElementById("sp1").innerText      = data.sp1.toFixed(3);
        document.getElementById("sp2").innerText      = data.sp2.toFixed(3);
        document.getElementById("desPos").innerText   = data.desPos.toFixed(3);
        document.getElementById("L").innerText        = data.L;
        document.getElementById("R").innerText        = data.R;
        document.getElementById("diff").innerText     = data.diff.toFixed(3);
        document.getElementById("manual").innerText   = data.manual;
        document.getElementById("fallen").innerText   = data.fallen;
        document.getElementById("heading").innerText = data.heading.toFixed(3);
        document.getElementById("desHeading").innerText = data.desiredHeading.toFixed(3);
        document.getElementById("avgSpeed").innerText   = data.avgSpeed.toFixed(3);
      } catch (e) {
        // ignore transient errors
      }
    }, 500);

    // 3) Set up the “Submit” handler (outside of bind())
    const posForm  = document.getElementById("posForm");
    const posInput = document.getElementById("posInput");
    const posAck   = document.getElementById("posAck");

    posForm.addEventListener("submit", async (e) => {
      e.preventDefault();               // prevent page reload
      const v = posInput.value;         // read the new set‐point
      try {
        const resp = await fetch("/set?val=" + encodeURIComponent(v));
        if (resp.ok) {
          posAck.innerText = "✔︎";      // success
          setTimeout(() => { posAck.innerText = ""; }, 1000);
        } else {
          posAck.innerText = "✘";      // error
          setTimeout(() => { posAck.innerText = ""; }, 1500);
        }
      } catch (err) {
        posAck.innerText = "✘";        // network failure
        setTimeout(() => { posAck.innerText = ""; }, 1500);
      }
    });
  });
</script>

</head>

<body>
  <div class="card">
    <h1>Balance-Bot Remote Controller</h1>

    <!-- ───── control pad ───── -->
    <div class="pad">
      <div></div>
      <button id="up">↑</button>
      <div></div>

      <button id="left">←</button>
      <div></div>
      <button id="right">→</button>

      <div></div>
      <button id="down">↓</button>
      <div></div>
    </div>

    <!-- ───── numeric set-point ───── -->
    <form id="posForm">
      <label>
        Desired position:
        <input id="posInput" type="number" step="0.01" name="val" value="0.00">
      </label>
      <input type="submit" value="Submit">
      <span id="posAck" style="margin-left:8px; color:green; font-size:0.9em;"></span>
    </form>

    <!-- ───── telemetry table ───── -->
    <table>
      <thead>
        <tr>
          <th>Parameter</th>
          <th>Value</th>
        </tr>
      </thead>
      <tbody>
        <tr><td>tiltSP</td>   <td class="value" id="tiltSP">0.000</td></tr>
        <tr><td>θ (theta)</td> <td class="value" id="theta">0.000</td></tr>
        <tr><td>posEst</td>   <td class="value" id="posEst">0.000</td></tr>
        <tr><td>w1pos</td>    <td class="value" id="w1pos">0.000</td></tr>
        <tr><td>w2pos</td>    <td class="value" id="w2pos">0.000</td></tr>
        <tr><td>sp1</td>      <td class="value" id="sp1">0.000</td></tr>
        <tr><td>sp2</td>      <td class="value" id="sp2">0.000</td></tr>
        <tr><td>desPos</td>   <td class="value" id="desPos">0.000</td></tr>
        <tr><td>LeftMode</td> <td class="value" id="L">0</td></tr>
        <tr><td>RightMode</td><td class="value" id="R">0</td></tr>
        <tr><td>diff</td>     <td class="value" id="diff">0.000</td></tr>
        <tr><td>manual</td>   <td class="value" id="manual">0</td></tr>
        <tr><td>fallen</td>   <td class="value" id="fallen">0</td></tr>
        <tr><td>heading (rad)</td><td class="value" id="heading">0.000</td></tr>
        <tr><td>desiredHeading (rad)</td><td class="value" id="desHeading">0.000</td></tr>
        <tr><td>avgSpeed</td>   <td class="value" id="avgSpeed">0.000</td></tr>
      </tbody>
    </table>
  </div>
</body>
</html>
)====";

/* ─────────────────── ROUTE HANDLERS ─────────────────────────── */
void handleRoot() {
  server.send_P(200, "text/html", HOMEPAGE);
}

void handleSet() {
  if (server.hasArg("val")) {
    g_webDesired = server.arg("val").toFloat();
    String resp = "Received: " + server.arg("val") + "<br><a href='/'>Back</a>";
    server.send(200, "text/html", resp);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleCmd() {
  if (!server.hasArg("act")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  if (fallen) {
    // If already fallen, ignore new drive commands
    server.send(200, "text/plain", "Ignored – robot has fallen");
    return;
  }

  String a = server.arg("act");
  /* ---- START commands ---- */


  float pos = 0.5f * (step1.getPositionRad() + step2.getPositionRad());

  if (a == "up_start") {
    g_webDesired = pos + 15.0f;
  }
  else if (a == "down_start") {
    g_webDesired = pos - 15.0f;
  }
  else if (a == "left_start")  {
    h_webDesired = spinComp - ROT_OFFSET;
  }
  else if (a == "right_start") {
    h_webDesired = spinComp + ROT_OFFSET;
  }
  else if (a == "right_stop" || a == "left_stop") {
    h_webDesired = spinComp;
  }
  else if (a == "down_stop" || a == "down_stop") {
    g_webDesired = pos;
  }
  else {
    server.send(400, "text/plain", "Unknown command");
    return;
  }

  server.send(200, "text/plain", "OK");
}

// Returns latest telemetry as JSON
void handleStatus() {
  server.send(200, "application/json", statusJSON);
}

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
  Serial.println("=== ESP32 Balance-Bot v3.2 (Web Telemetry) ===");

  /* Wi-Fi AP */
  WiFi.softAP("ESP32_BalanceBot", "12345678");
  Serial.print("Connect to http://");
  Serial.println(WiFi.softAPIP());

  /* Web routes */
  server.on("/",       handleRoot);
  server.on("/set",    handleSet);
  server.on("/cmd",    handleCmd);
  server.on("/status", handleStatus);
  server.begin();

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
  unsigned long now = millis();
  server.handleClient();  // handle web requests


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

    spinRate = g.gyro.roll + 0.031; 

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
    float heading = spinComp;
    float desiredHeading = h_webDesired;

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
      tiltSP, theta, heading, desiredHeading,
      centerPos, w1pos, w2pos, sp1, sp2,
      desiredOut, Lm, Rm, diffSp, mMode, fFlag, avgSpeed, gyro_z, gyro_y
    );

    // 4) Build JSON for /status, including desiredHeading
    statusJSON = "{";
    statusJSON += "\"tiltSP\":"        + String(tiltSP, 3)        + ",";
    statusJSON += "\"theta\":"         + String(theta, 3)         + ",";
    statusJSON += "\"heading\":"       + String(heading, 3)       + ",";
    statusJSON += "\"desiredHeading\":" + String(desiredHeading, 3)+ ",";
    statusJSON += "\"posEst\":"        + String(centerPos, 3)     + ",";
    statusJSON += "\"w1pos\":"         + String(w1pos, 3)         + ",";
    statusJSON += "\"w2pos\":"         + String(w2pos, 3)         + ",";
    statusJSON += "\"sp1\":"           + String(sp1, 3)           + ",";
    statusJSON += "\"sp2\":"           + String(sp2, 3)           + ",";
    statusJSON += "\"desPos\":"        + String(desiredOut, 3)    + ",";
    statusJSON += "\"L\":"             + String(Lm)               + ",";
    statusJSON += "\"R\":"             + String(Rm)               + ",";
    statusJSON += "\"diff\":"          + String(diffSp, 3)        + ",";
    statusJSON += "\"manual\":"        + String(mMode)            + ",";
    statusJSON += "\"fallen\":"        + String(fFlag)            + ",";
    statusJSON += "\"avgSpeed\":"      + String(avgSpeed, 3);
    statusJSON += "}";
  }

}
