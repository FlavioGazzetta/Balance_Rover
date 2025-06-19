/************************
 * ESP-NOW SLAVE – Balance-Bot firmware
 *   • Receives Packet objects from the web-UI master ESP32
 *   • Accepts either
 *        ▸ numeric set-points   (pkt.isSet == true)
 *        ▸ command strings      ("up_start", …)
 *        ▸ vision strings       "xCam,area"
 *   • Runs full balance control (inner 20 ms, outer 100 ms, print 500 ms)
 *   • (Optional) listens for the same "xCam,area" over UDP
 ************************/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <ctype.h>      // isdigit()
#include <string.h>
#include <math.h>

/* ─────────────────── Master’s MAC ─────────────────── */
uint8_t masterMac[6] = { 0xCC, 0x8D, 0xA2, 0x0C, 0x6C, 0xA4 };

/* ─────────────────── ESP-NOW packet ────────────────── */
typedef struct __attribute__((packed))
{
  bool  isSet;          // true → numeric set-point
  char  act[16];        // false → command OR "xCam,area"
  float val;
} Packet;

/* ───────── Vision globals (shared with control loop) ───────── */
volatile int   xCam    = 0;      // horizontal offset, px
volatile float areaCam = 0.0f;   // blob / bbox area, px²

/* ─────────────────── Optional UDP camera in ─────────────────── */
WiFiUDP        Udp;
#define LOCAL_UDP_PORT  5005
char           udpBuffer[32];
unsigned long  lastCamPacketMs = 0;
const unsigned long CAM_TIMEOUT = 500;   // ms without UDP ⇒ stale

/* ─────────────────── Pins & timing constants ─────────────────── */
const int STEPPER1_DIR_PIN   = 16;
const int STEPPER1_STEP_PIN  = 17;
const int STEPPER2_DIR_PIN   = 4;
const int STEPPER2_STEP_PIN  = 14;
const int STEPPER_EN_PIN     = 15;
const int TOGGLE_PIN         = 32;

const int PRINT_INTERVAL      = 500;      // ms
const int INNER_INTERVAL      = 20;       // ms
const int STEPPER_INTERVAL_US = 20;       // µs
const unsigned long OUTER_INTERVAL = 100; // ms

/* ─────────────────── Inner-loop (tilt) PID ─────────────────── */
const float Kp_inner        = 2000.0f;
const float Ki_inner        =    1.0f;
const float Kd_inner        =  200.0f;
const float c               =  0.96f;     // complementary-filter α
const float REFERENCE_ANGLE = -0.034f;    // rad

/* ─────────────────── Outer (position) constraints ───────────── */
const float ANGLE_CONSTRAINT   = 0.025f;   // rad
const float POSERRLIMIT        = 10.0f;
const float POSERRSLOWLIMIT    = 5.0f;

/* ─────────────────── Heading / spin PID ─────────────────────── */
const float ROT_OFFSET = 0.5f;  // rad (≈ 8.4°)
const float tp = 2.0f;          // P gain
const float td = 0.5f;          // D gain
const float ti = 0.1f;          // I gain

/* ─────────────────── Fall detection ─────────────────────────── */
const float FALL_THRESHOLD = 0.5f;  // rad (~28.6°)

/* ─────────────────── Objects ─────────────────────────────── */
ESP32Timer       ITimer(3);
Adafruit_MPU6050 mpu;
step             step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step             step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

/* ─────────────────── Shared state / flags ─────────────────── */
volatile float  g_webDesired = 0.0f;   // position set-point from master
volatile float  h_webDesired = 0.0f;   // heading set-point from master
volatile bool   fallen       = false;


volatile float rotpos        = 0.0f;
volatile float prev_rotpos   = 0.0f;
volatile float spinRate      = 0.0f;
volatile float spinErr       = 0.0f;
volatile float prev_spinErr  = 0.0f;
volatile float spinDeriv     = 0.0f;
volatile float spinIntegral  = 0.0f;
volatile float dt            = 0.0f;   // loop Δt for derivative

volatile float avgSpeed      = 0.0f;   // smoothed wheel speed

/* ─────────────────── Stepper ISR ───────────────────── */
bool IRAM_ATTR TimerHandler(void*)
{
  static bool tog = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, tog);
  tog = !tog;
  return true;
}

/* ─────────────── ESP-NOW RECEIVE CALLBACK ─────────────── */
void onSlaveRecv(const uint8_t*, const uint8_t* data, int len)
{
  if (len != sizeof(Packet)) return;
  Packet pkt;  memcpy(&pkt, data, sizeof(pkt));
  //Serial.print(pkt.act);

  /* —— Try "xCam,area" first —— */
  if (!pkt.isSet && (isdigit(pkt.act[0]) || pkt.act[0] == '-')) {
    char* endptr;
    long  xc = strtol(pkt.act, &endptr, 10);
    if (*endptr == ',') {                  // comma present
      float ar = atof(endptr + 1);
      xCam    = (int)xc;
      areaCam = ar;
      lastCamPacketMs = millis();
      return;
    }
  }

  /* —— Numeric position set-point —— */
  if (pkt.isSet) {
    g_webDesired = pkt.val;
    return;
  }

  /* —— Command strings —— */
  float pos = 0.5f * (step1.getPositionRad() + step2.getPositionRad());
  if      (!strcmp(pkt.act,"up_start"))    g_webDesired = pos - 15;
  else if (!strcmp(pkt.act,"down_start"))  g_webDesired = pos + 15;
  else if (!strcmp(pkt.act,"left_start"))  h_webDesired = rotpos - ROT_OFFSET;
  else if (!strcmp(pkt.act,"right_start")) h_webDesired = rotpos + ROT_OFFSET;
  else if (!strcmp(pkt.act,"left_stop")  ||
           !strcmp(pkt.act,"right_stop"))  h_webDesired = rotpos;
}

/* ─────────────── ESP-NOW SEND CALLBACK ─────────────── */
void onSlaveSend(const uint8_t*, esp_now_send_status_t s)
{
  if (s != ESP_NOW_SEND_SUCCESS) Serial.println("Status TX fail");
}

/* ─────────────────────────── SETUP ─────────────────────────── */
void setup()
{
  Serial.begin(115200);

  /* Wi-Fi connect (same SSID / pwd as master) */
  const char* ssid = "cole";
  const char* pwd  = "abcabcabc";
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print('.');
  }
  Serial.println("\nWi-Fi connected, channel " + String(WiFi.channel()));
  Serial.println("STA MAC: " + WiFi.macAddress());

  /* Optional UDP camera listener */
  Udp.begin(LOCAL_UDP_PORT);
  Serial.printf("UDP camera listener on port %u\n", LOCAL_UDP_PORT);

  /* Hardware init */
  pinMode(TOGGLE_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  if (!mpu.begin()) { Serial.println("MPU not found"); while (1) delay(1); }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Stepper ISR attach failed"); while (1) delay(1);
  }
  step1.setAccelerationRad(30.0f);
  step2.setAccelerationRad(30.0f);

  /* ESP-NOW init */
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (1) delay(1);
  }
  esp_now_register_recv_cb(onSlaveRecv);
  esp_now_register_send_cb(onSlaveSend);

  esp_now_peer_info_t peer{};  memcpy(peer.peer_addr, masterMac, 6);
  peer.channel = WiFi.channel();  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

bool wait = false;

float prevh = 0;
float prevg = 0;

/* ──────────────────────── MAIN LOOP ──────────────────────── */
void loop()
{
  unsigned long now = millis();

  /* ───── Optional UDP camera listener (non-blocking) ───── */
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(udpBuffer, sizeof(udpBuffer) - 1);
    if (len > 0) {
      udpBuffer[len] = '\0';
      char* token = strtok(udpBuffer, ",");
      if (token) {
        xCam = atoi(token);
        token = strtok(nullptr, ",");
        if (token) {
          areaCam = atof(token);
          lastCamPacketMs = now;
        }
      }
    }
  } else if (now - lastCamPacketMs > CAM_TIMEOUT) {
    wait = true;
  } else if (now - lastCamPacketMs < CAM_TIMEOUT){
    wait = false;
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

  /* ~~~~~ INNER CONTROL LOOP ~~~~~ */
  if (now - innerT >= INNER_INTERVAL) {
    innerT += INNER_INTERVAL;

    Serial.println(wait);

    int   xCamCentered = xCam - 640;   
    float deltaYaw     = -((xCamCentered/30.0)*(PI/180));
    if(!wait){
      h_webDesired = rotpos + deltaYaw;
      prevh = h_webDesired; 
    }
    else{
      h_webDesired = prevh;
    }   
    
    //Serial.println(areaCam);
    //Serial.println(xCam);
    
    // 1) Determine angle setpoint (reference) based on manual vs. auto
    ref = REFERENCE_ANGLE;
    

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

    spinRate = g.gyro.roll + 0.020; 

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

    float areapercent = areaCam / 1228800;

    float wheel1PosRad = step1.getPositionRad();
    float wheel2PosRad = step2.getPositionRad();
    float posEst = 0.5f * (wheel1PosRad + wheel2PosRad);

    

    if(!wait){
      g_webDesired = posEst + (areapercent - 0.6) * 1000;
      prevg = g_webDesired; 
    }
    else{
      g_webDesired = prevg;
    }

    /* ----- DRIVE WHEELS BASED ON MODE (unless fallen) ----- */
    if (fallen) {
      // If fallen, force both motors to zero speed
      step1.setTargetSpeedRad(0);
      step2.setTargetSpeedRad(0);
    }
    else {  // Straight‐line auto‐sync / “balance” mode
        
        rotpos      = prev_rotpos + spinRate * dt;
        prev_rotpos = rotpos;

        // 2) PID on heading (turn_reference set elsewhere)
        spinErr   = h_webDesired - rotpos;
        spinDeriv = (spinErr - prev_spinErr) / dt;
        spinIntegral  += spinErr * dt;

        float Prot       = tp * spinErr;
        float Drot       = td * spinDeriv;
        float Irot       = ti * spinIntegral;
        float rotvel = Prot + Drot + Irot;

        prev_spinErr = spinErr;

        // 3) Mix into your wheel commands alongside the balance‐drive (uout):
        step1.setTargetSpeedRad( uout  - rotvel );
        step2.setTargetSpeedRad( uout  + rotvel );

    }
  }

  /* ~~~~~ OUTER POSITION LOOP ~~~~~ */
  if (now - outerT >= OUTER_INTERVAL) {
    outerT += OUTER_INTERVAL;

    SumSpeed = 0;
    speedcount = 0;

    // 1) Read actual wheel positions in radians
    //    (average the two wheels → “robot center” rotation)
    float wheel1PosRad = step1.getPositionRad();
    float wheel2PosRad = step2.getPositionRad();
    float posEst = 0.5f * (wheel1PosRad + wheel2PosRad);

    // 3) Compute tiltSP based on manual vs. position control
    if (fallen) {
      // If fallen, lock tiltSP to zero so motors stay off
      tiltSP = 0;
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

        tiltSP = -constrain((Kp/abs(avgSpeed)), -ANGLE_CONSTRAINT, ANGLE_CONSTRAINT);


      }
      else if (absErr > POSERRSLOWLIMIT) {

        if(posErr < 0){

          Kp = (-0.025)/(2);

        }else{

          Kp = 0.025;

        }

        tiltSP = -constrain((Kp/abs(avgSpeed) * ((abs(posErr)-5)/10)), -ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT/2);

        }
      else{

        tiltSP = 0;

      }
      
    }

    avgSpeed = 0;
  }

  /* ~~~~~ DIAGNOSTICS PRINT & BUILD JSON (500 ms) ~~~~~ */
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
    float diffSp     = sp1 - sp2;
    int   fFlag      = fallen ? 1 : 0;

    // 3) Print to Serial, including desiredHeading
    Serial.printf(
      "tiltSP %.3f | θ %.3f | heading %.3f | desiredHeading %.3f | "
      "posEst %.3f | w1pos %.3f | w2pos %.3f | sp1 %.3f | sp2 %.3f | "
      "desPos %.3f | diff %.3f | fallen:%d | "
      "avgSpeed:%.3f | Roll %.3f | pitch %.3f\n",
      tiltSP, theta, rotpos, h_webDesired,
      centerPos, w1pos, w2pos, sp1, sp2,
      desiredOut, diffSp, fFlag, avgSpeed, gyro_z, gyro_y
    );
  }
}
