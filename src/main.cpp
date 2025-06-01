/********************************************************************
 * ESP32 Balance-Bot – combined firmware  (v3.2 • spin & auto-sync)
 *  • Single-loop inner-PID + outer-P controller  (exact gains)
 *  • Wi-Fi UI: ↑ ↓ ← → buttons (start/stop per button) + numeric set-point
 *  • ↑ / ↓ tilt while held (“freeze on release”)
 *  • ← / → spins in place (wheels opposite directions)
 *  • **Straight-line auto-sync:** wheels match speed whenever not spinning
 *  • Dynamic-Kd boost + complementary filter
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
const float c               =  0.96f;     // complementary-filter coefficient
const float REFERENCE_ANGLE = -0.045f;    // rad

// Kd boost thresholds
const float ERROR_SMALL_THRESHOLD = 0.005f;   // rad
const float GYRO_SPIKE_THRESHOLD  = 0.2f;     // rad/s
const float Kd_BOOST_FACTOR       = 100.0f;

// outer (position) loop gain
const float Kp_outer = 0.3f;

/* ───────────────────── MANUAL-DRIVE CONSTANTS ────────────────── */
const float TILT_MANUAL = 0.2f;  // rad  (≈ 11.5°)
const float SPIN_SPEED  = 4.0f;  // rad/s wheel-against-wheel

/* ─────────────── wheel *auto-sync* controller gains ──────────── */
const float SYNC_KP   = 5.0f;   // proportional on speed difference
const float SYNC_KI   = 6.0f;    // integral (small – it accumulates only mrad/s·s)
const float SYNC_MAX  = 10.0f;    // correction saturates at ±3 rad/s
const float SYNC_DEADBAND = 0.05f; // ignore differences smaller than 0.05 rad/s

/* ─────────────────────────── OBJECTS ─────────────────────────── */
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step             step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step             step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

/* ───────────────────────── WEB SERVER ────────────────────────── */
WebServer server(80);

/* ──────────────────── STATE / SHARED FLAGS ───────────────────── */
volatile float  g_webDesired       = 0.0f;   // numeric set-point

volatile bool   manualMode         = false;  // ↑ or ↓ held
volatile float  manualTiltOffset   = 0.0f;   // ±TILT_MANUAL

volatile bool   wheelLeftMode      = false;  // ← held
volatile bool   wheelRightMode     = false;  // → held

volatile bool   freezePosition     = false;  // hold current spot
volatile float  freezePositionPos  = 0.0f;   // latched pos

volatile float  g_positionEstimate = 0.0f;   // shared with /cmd

/* ─────────────────────── HTML HOMEPAGE ───────────────────────── */
const char HOMEPAGE[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  *{ -webkit-tap-highlight-color:transparent; }
  .pad{display:flex;flex-direction:column;align-items:center;width:220px;margin:auto;}
  .row{display:flex;justify-content:center;}
  button{
    width:64px;height:64px;font-size:36px;margin:6px;border-radius:12px;
    background:#e0e0e0;color:#000;border:1px solid #999;
    user-select:none;-webkit-user-select:none;touch-action:manipulation;
    transition:background .05s,color .05s;
  }
  button:active,button.pressed{background:#3f51b5;color:#fff;}
</style>
<script>
function send(cmd){ fetch('/cmd?act='+cmd); }

/* bind(id, startCmd, stopCmd) */
function bind(id,start,stop){
  const el=document.getElementById(id); let rpt=null;
  const down=e=>{
    e.preventDefault();
    el.classList.add('pressed');
    send(start);
    rpt=setInterval(()=>send(start),200);
  };
  const up=e=>{
    e.preventDefault();
    if(rpt){clearInterval(rpt); rpt=null;}
    el.classList.remove('pressed');
    send(stop);
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
  <h2 style="text-align:center;">ESP32&nbsp;Balance-Bot</h2>
  <div class="pad">
    <div class="row"><button id="up">&uarr;</button></div>
    <div class="row">
      <button id="left">&larr;</button>
      <button id="right">&rarr;</button>
    </div>
    <div class="row"><button id="down">&darr;</button></div>
  </div>
  <hr>
  <form action="/set" method="GET" style="text-align:center;">
    Desired&nbsp;position:
    <input type="number" step="0.01" name="val" value="0.00">
    <input type="submit" value="Submit">
  </form>
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
  }else{
    server.send(400,"text/plain","Bad Request");
  }
}

void handleCmd(){
  if(!server.hasArg("act")){
    server.send(400,"text/plain","Bad Request"); return;
  }
  String a = server.arg("act");

  /* ---- START commands ---- */
  if      (a=="up_start")    { manualMode=true;  manualTiltOffset= TILT_MANUAL;  freezePosition=false; }
  else if (a=="down_start")  { manualMode=true;  manualTiltOffset=-TILT_MANUAL;  freezePosition=false; }
  else if (a=="left_start")  { wheelLeftMode=true;  wheelRightMode=false; freezePosition=false; }
  else if (a=="right_start") { wheelRightMode=true; wheelLeftMode=false;  freezePosition=false; }

  /* ---- STOP commands ---- */
  else if (a=="up_stop" || a=="down_stop"){
    manualTiltOffset=0; manualMode=false;
    freezePosition=true;      // hold wherever we are
    freezePositionPos=g_positionEstimate;
  }
  else if (a=="left_stop"){
    wheelLeftMode=false;
    freezePosition=true;
    freezePositionPos=g_positionEstimate;
  }
  else if (a=="right_stop"){
    wheelRightMode=false;
    freezePosition=true;
    freezePositionPos=g_positionEstimate;
  }
  else{ server.send(400,"text/plain","Unknown command"); return; }

  server.send(200,"text/plain","OK");
}

/* ───────────────────── STEPPER TIMER ISR ─────────────────────── */
bool IRAM_ATTR TimerHandler(void*){
  static bool tog=false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN,tog);
  tog = !tog;
  return true;
}

/* ────────────────────────── SETUP ────────────────────────────── */
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 Balance-Bot v3.2 ===");

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
  pinMode(TOGGLE_PIN,OUTPUT);
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN,LOW);

  if(!mpu.begin()){
    Serial.println("MPU6050 not detected! Halt.");
    while(1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if(!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)){
    Serial.println("Failed to attach stepper ISR");
    while(1) delay(10);
  }

  step1.setAccelerationRad(15.0f);
  step2.setAccelerationRad(15.0f);
}

/* ────────────────────────── MAIN LOOP ───────────────────────── */
void loop(){
  unsigned long now = millis();
  server.handleClient();                       // non-blocking

  /* --------------- INNER LOOP (20 ms) ---------------- */
  static unsigned long prevTime = now, innerT = 0;
  static float theta = 0.0f, integral = 0.0f, uout = 0.0f;
  static float tilt_acc_z = 0.0f, gyro_y = 0.0f, err_inner = 0.0f;

  /* sync PI integrator */
  static float syncInt = 0.0f;

  /* --------------- OUTER LOOP (200 ms) --------------- */
  static unsigned long outerT = 0;
  static float posEst = 0.0f, desiredPos = 0.0f, tiltSP = 0.0f;

  /* --------------- Diagnostics (500 ms) --------------- */
  static unsigned long printT = 0;

  /* ~~~~~~~~~~~~~ INNER CONTROL LOOP ~~~~~~~~~~~~~ */
  if(now - innerT >= INNER_INTERVAL){
    innerT += INNER_INTERVAL;

    /* wheel speed → position estimate (rad) */
    float sp1_meas = step1.getSpeedRad();
    float sp2_meas = step2.getSpeedRad();
    posEst += 0.5f * (sp1_meas + sp2_meas) * (INNER_INTERVAL / 1000.0f);
    g_positionEstimate = posEst;

    /* IMU */
    sensors_event_t a,g,tmp;
    mpu.getEvent(&a,&g,&tmp);
    tilt_acc_z = a.acceleration.z / 9.81f;
    gyro_y     = g.gyro.y;

    /* complementary filter */
    float dt = (now - prevTime) / 1000.0f;
    prevTime = now;
    theta = (1.0f - c)*tilt_acc_z + c*(theta + gyro_y*dt);

    /* error */
    if(fabsf(REFERENCE_ANGLE - theta) < 0.01f)
         err_inner = (REFERENCE_ANGLE + tiltSP) - theta;
    else err_inner = (REFERENCE_ANGLE) - theta;

    integral += err_inner * dt;

    float deriv = -gyro_y;
    float Kd_eff = Kd_inner;
    if((fabsf(err_inner) < ERROR_SMALL_THRESHOLD) &&
       (fabsf(gyro_y)    > GYRO_SPIKE_THRESHOLD)){
      Kd_eff *= Kd_BOOST_FACTOR;
    }

    uout = Kp_inner*err_inner + Ki_inner*integral + Kd_eff*deriv;

    /* drive wheels */
    if(wheelLeftMode){            // ← : anticlockwise spin
      step1.setTargetSpeedRad(uout + SPIN_SPEED);   // left wheel fwd
      step2.setTargetSpeedRad(uout - SPIN_SPEED);   // right wheel rev
    }
    else if(wheelRightMode){      // → : clockwise spin
      step1.setTargetSpeedRad(uout - SPIN_SPEED);   // left wheel rev
      step2.setTargetSpeedRad(uout + SPIN_SPEED);   // right wheel fwd
    }
    else{                         // straight drive – apply auto-sync
      /* PI controller on measured speed difference */
      float diff = sp1_meas - sp2_meas;             // +ve = left faster
      float corr = SYNC_KP * diff;
      if(abs(diff) > SYNC_DEADBAND){
        /* within dead-band – softly bleed integral */
        step1.setTargetSpeedRad(uout + corr);  // left
        step2.setTargetSpeedRad(uout - corr);  // right
      }

      step1.setTargetSpeedRad(uout);  // left
      step2.setTargetSpeedRad(uout);  // right

      
    }
  }

  /* ~~~~~~~~~~~~~ OUTER POSITION LOOP ~~~~~~~~~~~~~ */
  if(now - outerT >= OUTER_INTERVAL){
    outerT += OUTER_INTERVAL;

    if(manualMode){
      tiltSP = manualTiltOffset;            // ↑/↓ override
    }else{
      desiredPos = freezePosition ? freezePositionPos : g_webDesired;
      float posErr = desiredPos - posEst;
      tiltSP = constrain(Kp_outer*posErr, -0.187f, 0.013f); // (≈ ±0.1-0.087)
    }
  }

  /* ~~~~~~~~~~~~~ DIAGNOSTICS PRINT ~~~~~~~~~~~~~ */
  if(now - printT >= PRINT_INTERVAL){
    printT += PRINT_INTERVAL;
    Serial.printf(
      "θ_ref %.3f | tiltSP %.3f | θ %.3f | pos %.3f | sp1 %.3f | sp2 %.3f | desPos %.3f | L %d | R %d | diff %.3f\n",
      REFERENCE_ANGLE, tiltSP, theta, posEst,
      step1.getSpeedRad(), step2.getSpeedRad(),
      desiredPos, wheelLeftMode, wheelRightMode, step1.getSpeedRad() - step2.getSpeedRad()
    );
  }
}
