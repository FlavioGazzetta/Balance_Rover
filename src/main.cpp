/********************************************************************
 * ESP32 Balance-Bot – combined firmware
 *  • Single-loop inner-PID + outer-P controller (exact values)
 *  • Wi-Fi UI: Forward / Back / Stop buttons + numeric set-point
 *  • Manual-drive tilt offset with “freeze on release”
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

/* ---------------------------------------------------------------
 *                         PIN MAP
 * --------------------------------------------------------------*/
const int STEPPER1_DIR_PIN   = 16;
const int STEPPER1_STEP_PIN  = 17;
const int STEPPER2_DIR_PIN   = 4;
const int STEPPER2_STEP_PIN  = 14;
const int STEPPER_EN_PIN     = 15;
const int TOGGLE_PIN         = 32;

/* ---------------------------------------------------------------
 *                         TIMING
 * --------------------------------------------------------------*/
const int PRINT_INTERVAL        = 500;      // ms
const int INNER_INTERVAL        = 20;       // ms
const int STEPPER_INTERVAL_US   = 20;       // µs
const unsigned long OUTER_INTERVAL = 200;   // ms

/* ---------------------------------------------------------------
 *                CONTROLLER – keep EXACTLY these values
 * --------------------------------------------------------------*/
const float Kp_inner        = 1000.0f;
const float Ki_inner        =    1.0f;
const float Kd_inner        =  200.0f;
const float c               =  0.96f;     // complementary-filter coefficient
const float REFERENCE_ANGLE = -0.045f;    // rad

// --- dynamic Kd boost thresholds ---
const float ERROR_SMALL_THRESHOLD = 0.005f;   // rad
const float GYRO_SPIKE_THRESHOLD  = 0.2f;     // rad/s
const float Kd_BOOST_FACTOR       = 100.0f;

// --- outer (position) loop gain ---
const float Kp_outer = 0.3f;

/* ---------------------------------------------------------------
 *                MANUAL-DRIVE (Forward / Back)
 * --------------------------------------------------------------*/
const float TILT_MANUAL = 0.05f; // rad ≈ 0.57° additional tilt

/* ---------------------------------------------------------------
 *                         OBJECTS
 * --------------------------------------------------------------*/
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step             step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step             step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

/* ---------------------------------------------------------------
 *                     WEB-SERVER (port 80)
 * --------------------------------------------------------------*/
WebServer server(80);

// — global set-point from the numeric form —
volatile float g_webDesired = 0.0f;

// — flags shared with the /cmd endpoint —
volatile bool  manualMode        = false;   // true while Fwd/Back held
volatile float manualTiltOffset  = 0.0f;    // ±TILT_MANUAL
volatile bool  freezePosition    = false;   // true after “Stop” released
volatile float freezePositionPos = 0.0f;    // position value at freeze

// — expose position to the /cmd handler so that “Stop” can latch it —
volatile float g_positionEstimate = 0.0f;

/* ----------------------------------------------------------------
 *  HTML PAGE  – three-button UI with active-state colours
 * ----------------------------------------------------------------*/
const char HOMEPAGE[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  *{ -webkit-tap-highlight-color:transparent; }
  button{
    width:92px;height:54px;font-size:18px;margin:6px;border-radius:8px;
    background-color:#e0e0e0;color:#000;border:1px solid #999;
    user-select:none;-webkit-user-select:none;-ms-user-select:none;
    touch-action:manipulation;
    transition:background-color .05s,color .05s;
  }
  button:active,
  button.pressed{
    background-color:#3f51b5;   /* deep-blue when active */
    color:#fff;
  }
</style>
<script>
/* minimal fetch helper */
function send(cmd){ fetch('/cmd?act='+cmd); }

/* -----------------------------------------------------------
   bind(id,start)
   • fires command immediately on press
   • keeps sending the command every 200 ms while held
   • on release: cancels timer + sends "stop"
   ----------------------------------------------------------- */
function bind(id,start){
  const el=document.getElementById(id);
  let repeat=null;

  const down=e=>{
    e.preventDefault();
    el.classList.add('pressed');
    send(start);
    repeat=setInterval(()=>send(start),200);
  };

  const up=e=>{
    e.preventDefault();
    if(repeat){ clearInterval(repeat); repeat=null; }
    el.classList.remove('pressed');
    send('stop');
  };

  el.addEventListener('pointerdown',down,{passive:false});
  el.addEventListener('pointerup'  ,up  ,{passive:false});
  el.addEventListener('pointercancel',up,{passive:false});
  el.addEventListener('pointerleave' ,up,{passive:false});
}

window.addEventListener('load',()=>{
  bind('btnBack','back_start');
  bind('btnFwd' ,'fwd_start');
});
</script>
</head>
<body>
  <h2>ESP32&nbsp;Balance-Bot</h2>

  <button id="btnBack">Backward</button><br>
  <button id="btnFwd">Forward</button>

  <hr>
  <form action="/set" method="GET">
    Desired&nbsp;position:
    <input type="number" step="0.01" name="val" value="0.00">
    <input type="submit" value="Submit">
  </form>
</body>
</html>
)====";

/* ---------------- URL handlers ---------------- */
void handleRoot()                { server.send_P(200,"text/html",HOMEPAGE); }

void handleSet() {
  if (server.hasArg("val")) {
    g_webDesired = server.arg("val").toFloat();
    String resp  = "Received: " + server.arg("val") + "<br><a href='/'>Back</a>";
    server.send(200,"text/html",resp);
  } else {
    server.send(400,"text/plain","Bad Request");
  }
}

void handleCmd() {
  if (!server.hasArg("act")) {
    server.send(400,"text/plain","Bad Request");
    return;
  }
  String a = server.arg("act");
  if      (a=="fwd_start")  { manualMode=true;  manualTiltOffset= TILT_MANUAL; freezePosition=false; }
  else if (a=="back_start") { manualMode=true;  manualTiltOffset= -TILT_MANUAL; freezePosition=false; }
  else {
    manualTiltOffset=0;
    manualMode=false;
    freezePosition=true;
    freezePositionPos=g_positionEstimate;
  }
  server.send(200,"text/plain","OK");
}

/* ----------------  Stepper ISR  ---------------- */
bool IRAM_ATTR TimerHandler(void*) {
  static bool tog=false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN,tog);
  tog=!tog;
  return true;
}

/* =====================  SETUP  ===================== */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 Balance-Bot – Combined Firmware ===");

  /* ---------- Wi-Fi Soft-AP ---------- */
  WiFi.softAP("ESP32_BalanceBot","12345678");
  Serial.print("Connect to http://");
  Serial.println(WiFi.softAPIP());

  /* ---------- Web-server routes ---------- */
  server.on("/",    handleRoot);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
  server.begin();

  /* ---------- Hardware init ---------- */
  pinMode(TOGGLE_PIN,OUTPUT);
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN,LOW);

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

/* =====================  MAIN LOOP  ===================== */
void loop() {
  unsigned long now = millis();

  server.handleClient();                 // non-blocking

  /* --- INNER LOOP ------------------------------------- */
  static unsigned long prevTime = now, innerT = 0;
  static float theta = 0.0f, integral = 0.0f, uout = 0.0f;
  static float tilt_acc_z = 0.0f, gyro_y = 0.0f, err_inner = 0.0f;

  /* --- OUTER LOOP ------------------------------------- */
  static unsigned long outerT = 0;
  static float posEst = 0.0f, desiredPos = 0.0f, tiltSP = 0.0f;

  /* --- Diagnostics ----------------------------------- */
  static unsigned long printT = 0;

  /* ----------------- INNER ----------------- */
  if (now - innerT >= INNER_INTERVAL) {
    innerT += INNER_INTERVAL;

    /* wheel speed → position estimate (rad) */
    float sp1 = step1.getSpeedRad();
    float sp2 = step2.getSpeedRad();
    posEst += 0.5f * (sp1 + sp2) * (INNER_INTERVAL / 1000.0f);
    g_positionEstimate = posEst;       // publish for /cmd “stop”

    /* IMU */
    sensors_event_t a, g, tmp;
    mpu.getEvent(&a, &g, &tmp);
    tilt_acc_z = a.acceleration.z / 9.81f;
    gyro_y     = g.gyro.y;

    /* complementary filter */
    float dt = (now - prevTime) / 1000.0f;
    prevTime = now;
    theta = (1.0f - c) * tilt_acc_z + c * (theta + gyro_y * dt);

    /* error */
    if (fabsf(REFERENCE_ANGLE - theta) < 0.01f)
         err_inner = (REFERENCE_ANGLE + tiltSP) - theta;
    else err_inner = (REFERENCE_ANGLE) - theta;

    integral += err_inner * dt;

    float deriv = -gyro_y;
    float Kd_eff = Kd_inner;
    if ((fabsf(err_inner) < ERROR_SMALL_THRESHOLD) &&
        (fabsf(gyro_y)    > GYRO_SPIKE_THRESHOLD)) {
      Kd_eff *= Kd_BOOST_FACTOR;
    }

    uout = Kp_inner * err_inner + Ki_inner * integral + Kd_eff   * deriv;

    step1.setTargetSpeedRad(uout);
    step2.setTargetSpeedRad(uout);
  }

  /* ----------------- OUTER ----------------- */
  if (now - outerT >= OUTER_INTERVAL) {
    outerT += OUTER_INTERVAL;

    if (manualMode) {
      tiltSP = manualTiltOffset;             // override while button held
    } else {
      desiredPos = freezePosition ? freezePositionPos : g_webDesired;
      float posErr = desiredPos - posEst;
      tiltSP = constrain(Kp_outer * posErr, -0.1-0.087, 0.1-0.087);
    }
  }

  /* --------------- DIAGNOSTICS --------------- */
  if (now - printT >= PRINT_INTERVAL) {
    printT += PRINT_INTERVAL;
    Serial.printf("θ_ref %.3f | tiltSP %.3f | θ %.3f | pos %.3f | sp1 %.3f | sp2 %.3f | desPos %.3f | man %d | freeze %d\n",
                  REFERENCE_ANGLE, tiltSP, theta, posEst,
                  step1.getSpeedRad(), step2.getSpeedRad(),
                  desiredPos, manualMode, freezePosition);
  }
}
