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
const unsigned long OUTER_INTERVAL = 200;   // ms

/* ───────────────────── PID & FILTER CONSTANTS ────────────────── */
const float Kp_inner        = 1000.0f;
const float Ki_inner        =    1.0f;
const float Kd_inner        =  200.0f;
const float c               =  0.96f;     // complementary‐filter coefficient
const float REFERENCE_ANGLE = -0.035f;    // rad

// Kd boost thresholds
const float ERROR_SMALL_THRESHOLD = 0.005f;   // rad
const float GYRO_SPIKE_THRESHOLD  = 0.2f;     // rad/s
const float Kd_BOOST_FACTOR       = 100.0f;

// outer (position) loop gain
const float Kp_outer = 0.3f;

/* ───────────────────── MANUAL‐DRIVE CONSTANTS ────────────────── */
const float TILT_MANUAL = 1.0f; 
const float TILT_ANGLE  = 0.015f;
const float SPIN_SPEED  = 3.0f;  // rad/s wheel‐against‐wheel

/* ─────────────── wheel *auto‐sync* controller gains ──────────── */
const float SYNC_KP       = 5.0f;    // proportional on speed difference
const float SYNC_KI       = 6.0f;    // integral (small – accumulates only mrad/s·s)
const float SYNC_MAX      = 10.0f;   // correction saturates at ±10 rad/s
const float SYNC_DEADBAND = 0.05f;   // ignore speed diffs < 0.05 rad/s

/* ─────────────────────────────────────────────────────────────────
   ───  “manual‐mode speed limiter” constants (unchanged)     𐄂
   ───────────────────────────────────────────────────────────────── */
const float MAX_AVG_SPEED_RAD   = 10.0f;  // rad/s
const float SPEED_REDUCTION_RAD =  1.0f;  // rad/s
/* ─────────────────────────────────────────────────────────────────
   ───────────────────────────────────────────────────────────────── */

/* ─────────────────────────── FALL‐DETECTION ───────────────────── */
const float FALL_THRESHOLD = 0.5f;  // radians (~28.6°). Adjust as needed
volatile bool fallen = false;       // set true once robot “falls”


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
<title>Remote-Controller</title>

<!-- ——————  styles —————— -->
<style>
  :root{
    --primary:#3f51b5;--primary-dark:#303f9f;--surface:#ffffffee;
    --surface-hover:#f1f3f7;--surface-press:var(--primary);
    --text-main:#2b2b2b;--text-muted:#666;--radius:14px;
    --shadow:0 4px 12px rgba(0,0,0,.14);
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}
  html,body{height:100%;margin:0;font-family:-apple-system,BlinkMacSystemFont,
            "Segoe UI",Roboto,Helvetica,Arial,sans-serif;
            background:linear-gradient(135deg,#d7e1ec 0%,#f6f7f9 100%);}
  body{display:flex;align-items:center;justify-content:center;padding:14px;}

  .card{
    background:var(--surface);padding:22px 26px;border-radius:var(--radius);
    box-shadow:var(--shadow);max-width:320px;width:100%;
    display:flex;flex-direction:column;gap:22px;
  }
  h1{font-size:1.5rem;font-weight:600;margin:0;text-align:center;
     color:var(--primary-dark);}

  /* ---------- control pad (perfect cross) ---------- */
  .pad{
    display:grid;grid-template-columns:repeat(3,1fr);
    grid-template-rows:repeat(3,1fr);gap:14px;
    justify-items:center;align-items:center;
  }
  #up   {grid-column:2;grid-row:1;}
  #left {grid-column:1;grid-row:2;}
  #right{grid-column:3;grid-row:2;}
  #down {grid-column:2;grid-row:3;}

  button{
    width:80px;height:80px;border:none;font-size:38px;font-weight:600;
    border-radius:50%;background:var(--surface-hover);color:var(--text-main);
    cursor:pointer;touch-action:manipulation;
    transition:background .08s,color .08s,transform .08s;
    user-select:none;-webkit-user-select:none;-moz-user-select:none;
    -ms-user-select:none;-webkit-touch-callout:none;
  }
  button:active,
  button.pressed{
    background:var(--surface-press);color:#fff;transform:scale(0.96);
  }

  /* ---------- numeric set-point form ---------- */
  form{display:flex;flex-direction:column;gap:12px;text-align:center;}
  label{font-size:.88rem;color:var(--text-muted);}
  input[type=number]{
    padding:8px 10px;border:1px solid #ccd1d8;border-radius:var(--radius);
    width:120px;margin:auto;font-size:1rem;text-align:center;
  }
  input[type=submit]{
    padding:10px;border:none;border-radius:var(--radius);
    background:var(--primary);color:#fff;font-size:1rem;font-weight:600;
    cursor:pointer;transition:background .1s;
  }
  input[type=submit]:hover{background:var(--primary-dark);}
</style>

<!-- ——————  logic —————— -->
<script>
function send(cmd){fetch('/cmd?act='+cmd);}
function bind(id,start,stop){
  const el=document.getElementById(id);let rpt=null;
  const down=e=>{
    e.preventDefault();el.classList.add('pressed');
    send(start);rpt=setInterval(()=>send(start),200);
  };
  const up=e=>{
    e.preventDefault();el.classList.remove('pressed');
    clearInterval(rpt);rpt=null;send(stop);
  };
  el.addEventListener('pointerdown',down,{passive:false});
  ['pointerup','pointercancel','pointerleave'].forEach(ev=>
    el.addEventListener(ev,up,{passive:false}));
}
window.addEventListener('load',()=>{
  bind('up'   ,'up_start'   ,'up_stop');
  bind('down' ,'down_start' ,'down_stop');
  bind('left' ,'left_start' ,'left_stop');
  bind('right','right_start','right_stop');
});
</script>
</head>

<body>
  <div class="card">
    <h1>Remote-Controller</h1>

    <!-- control pad ----------------------------------------------------------- -->
    <div class="pad">
      <button id="up">↑</button>
      <button id="left">←</button>
      <button id="right">→</button>
      <button id="down">↓</button>
    </div>

    <!-- numeric set-point ----------------------------------------------------- -->
    <form action="/set" method="GET">
      <label>
        Desired&nbsp;position
        <input type="number" step="0.01" name="val" value="0.00">
      </label>
      <input type="submit" value="Submit">
    </form>
  </div>
</body>
</html>
)====";

/* ─────────────────── ROUTE HANDLERS ─────────────────────────── */
void handleRoot() { server.send_P(200,"text/html",HOMEPAGE); }

void handleSet(){
  if(server.hasArg("val")){
    g_webDesired = server.arg("val").toFloat();
    String resp  = "Received: " + server.arg("val") + "<br><a href='/'>Back</a>";
    server.send(200,"text/html",resp);
  } else {
    server.send(400,"text/plain","Bad Request");
  }
}

void handleCmd(){
  if(!server.hasArg("act")){
    server.send(400,"text/plain","Bad Request"); 
    return;
  }
  if (fallen) {
    // If already fallen, ignore any new drive commands
    server.send(200,"text/plain","Ignored – robot has fallen");
    return;
  }

  String a = server.arg("act");

  /* ---- START commands ---- */
  if      (a=="up_start")    { 
    manualMode = true;  
    movementoffset =  TILT_MANUAL;  
    manualTiltOffset =  TILT_ANGLE;   
    freezePosition = false;  
  }
  else if (a=="down_start")  { 
    manualMode = true;  
    movementoffset = -TILT_MANUAL;  
    manualTiltOffset = -TILT_ANGLE;  
    freezePosition = false;  
  }
  else if (a=="left_start")  { 
    wheelLeftMode = true;  
    wheelRightMode = false; 
    freezePosition = false; 
  }
  else if (a=="right_start") { 
    wheelRightMode = true; 
    wheelLeftMode = false;  
    freezePosition = false; 
  }

  /* ---- STOP commands ---- */
  else if (a=="up_stop" || a=="down_stop"){
    movementoffset = 0; 
    manualTiltOffset = 0; 
    manualMode = false;
    freezePosition = true;         // latch current position
    // We’ll set freezePositionPos in loop() on next outer‐tick
  }
  else if (a=="left_stop"){
    wheelLeftMode = false;
    freezePosition = true;
    step1.setTargetSpeedRad(0);  
    step2.setTargetSpeedRad(0);  
    delay(40);
  }
  else if (a=="right_stop"){
    wheelRightMode = false;
    freezePosition = true;
    step1.setTargetSpeedRad(0);  
    step2.setTargetSpeedRad(0);  
    delay(40);
  }
  else { 
    server.send(400,"text/plain","Unknown command"); 
    return; 
  }

  server.send(200,"text/plain","OK");
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
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 Balance-Bot v3.2 (With fall detection & recovery) ===");

  /* Wi-Fi AP */
  WiFi.softAP("ESP32_BalanceBot","12345678");
  Serial.print("Connect to http://");
  Serial.println(WiFi.softAPIP());

  /* Web routes */
  server.on("/",    handleRoot);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
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

  step1.setAccelerationRad(15.0f);
  step2.setAccelerationRad(15.0f);
}

/* ────────────────────────── MAIN LOOP ───────────────────────── */
void loop(){
  unsigned long now = millis();
  server.handleClient();  // handle web requests

  /* --------------- INNER LOOP (20 ms) ---------------- */
  static unsigned long prevTime = now, innerT = 0;
  static float theta = 0.0f, integral = 0.0f, uout = 0.0f;
  static float tilt_acc_z = 0.0f, gyro_y = 0.0f, err_inner = 0.0f;

  /* sync PI integrator */
  static float syncInt = 0.0f;

  /* --------------- OUTER LOOP (200 ms) --------------- */
  static unsigned long outerT = 0;
  static float desiredPos = 0.0f, tiltSP = 0.0f;

  /* --------------- Diagnostics (500 ms) --------------- */
  static unsigned long printT = 0;

  float ref = 0;

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
    gyro_y     = g.gyro.y;

    float dt = (now - prevTime) / 1000.0f;  // in seconds
    prevTime = now;
    theta = (1.0f - c) * tilt_acc_z + c * (theta + gyro_y * dt);

    // ─── FALL DETECTION & RECOVERY ─────────────────────────────────
    if (!fallen && fabsf(theta) > FALL_THRESHOLD) {
      // Just detected a fall
      fallen = true;
      Serial.println("Rover has fallen!");
    }
    else if (fallen && fabsf(theta) <= FALL_THRESHOLD) {
      // Tilt is back under threshold → clear fallen flag
      fallen = false;
      Serial.println("Rover recovered, resuming operation");
      // (Optionally, you could zero‐out integral here if you want a fresh start:
      // integral = 0.0f;
      // )
    }

    // 4) Inner‐loop error & integral/derivative
    
    err_inner = (ref + tiltSP) - theta;
    
    

    integral += err_inner * dt;
    float deriv = -gyro_y;
    float Kd_eff = Kd_inner;
    if ((fabsf(err_inner) < ERROR_SMALL_THRESHOLD) &&
        (fabsf(gyro_y)    > GYRO_SPIKE_THRESHOLD) &&
        !manualMode) {
      Kd_eff *= Kd_BOOST_FACTOR;
    }

    uout = Kp_inner * err_inner
         + Ki_inner * integral
         + Kd_eff * deriv;


    /* ----- DRIVE WHEELS BASED ON MODE (unless fallen) ----- */
    if (fallen) {
      // If fallen, force both motors to zero speed
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
    }
    else if (wheelLeftMode) {            // ← spin in place
      step1.setTargetSpeedRad(uout + SPIN_SPEED);  
      step2.setTargetSpeedRad(uout - SPIN_SPEED);  
    }
    else if (wheelRightMode) {      // → spin in place
      step1.setTargetSpeedRad(uout - SPIN_SPEED);
      step2.setTargetSpeedRad(uout + SPIN_SPEED);
    }
    else if (manualMode) {
      // ─── “manual‐mode speed limiter” (unchanged) ───
      float avgSpeed = 0.5f * (sp1_meas + sp2_meas);
      if (avgSpeed > MAX_AVG_SPEED_RAD) {
        uout -= SPEED_REDUCTION_RAD;
        if (uout < MAX_AVG_SPEED_RAD) {
          uout = MAX_AVG_SPEED_RAD;
        }
      }
      // ──────────────────────────────────────────────

      // Both wheels in tandem under manual tilt:
      step1.setTargetSpeedRad(uout);
      step2.setTargetSpeedRad(uout);
    }
    else {  // Straight‐line auto‐sync / “balance” mode
      step1.setTargetSpeedRad(uout);
      step2.setTargetSpeedRad(uout);
      // (Auto‐sync PI on speed difference would go here if desired)
    }
  }


  /* ~~~~~~~~~~~~~ OUTER POSITION LOOP ~~~~~~~~~~~~~ */
  if (now - outerT >= OUTER_INTERVAL) {
    outerT += OUTER_INTERVAL;

    // 1) Read actual wheel positions in radians
    //    (average the two wheels → “robot center” rotation)
    float wheel1PosRad = step1.getPositionRad();
    float wheel2PosRad = step2.getPositionRad();
    float posEst = 0.5f * (wheel1PosRad + wheel2PosRad);

    // 2) If we just left manual mode, latch freezePositionPos:
    if (!manualMode && freezePosition) {
      // Once we drop out of manual, we want to hold wherever we were
      freezePositionPos = posEst;
      // Keep freezePosition = true until user issues a new setpoint
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
      float desired = freezePosition ? freezePositionPos : g_webDesired;
      float posErr  = (desired - posEst)*0.1;
      tiltSP = - constrain(Kp_outer * posErr, -0.018f, 0.018f);
    }
  }


  /* ~~~~~~~~~~~~~ DIAGNOSTICS PRINT ~~~~~~~~~~~~~ */
  if (now - printT >= PRINT_INTERVAL) {
    printT += PRINT_INTERVAL;
    float w1pos = step1.getPositionRad();
    float w2pos = step2.getPositionRad();
    Serial.printf(
      "tiltSP %.3f | θ %.3f | posEst %.3f | w1pos %.3f | w2pos %.3f | sp1 %.3f | sp2 %.3f | desPos %.3f | L %d | R %d | diff %.3f | manual:%d | fallen:%d\n",
      tiltSP,
      theta,
      0.5f * (w1pos + w2pos),
      w1pos,
      w2pos,
      step1.getSpeedRad(),
      step2.getSpeedRad(),
      (freezePosition ? freezePositionPos : g_webDesired),
      wheelLeftMode,
      wheelRightMode,
      step1.getSpeedRad() - step2.getSpeedRad(),
      manualMode,
      fallen ? 1 : 0
    );
  }
}
