/******************************************************************
  BatteryMonitor_SH1106_U8g2.ino
  ---------------------------------------------------------------
  ‣ ESP32-DevKit (on-chip ADC only)
       Vo   sensor  → GPIO39  (ADC1_CH3)
       Vbatt sensor → GPIO34  (ADC1_CH6)
  ‣ 1.3" 128×64 OLED with SH1106 controller
       VCC → 3V3   |  GND → GND
       SCK → GPIO22|  SDA → GPIO21
  ‣ I_logic = ((Vo  - 0.9)  / 510) / 0.01
    I_motor = ((Vbatt - 1.27) / 100) / 0.01
  ‣ Charge integrated with trapezoidal rule
  ‣ Screen layout
       ┌─────────────┐
       │  BAT ICON    │
       │  + % text    │
       │──────────────│
       │   P = XX.X W │
       │              │
       └──────────────┘
******************************************************************/

#include <Wire.h>
#include <U8g2lib.h>

// ------- OLED (SH1106) -------
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ------- ADC pins & constants -------
const int   PIN_VO   = 39;   // GPIO39 (your Vo sensor)
const int   PIN_VBAT = 34;   // GPIO34 (your Vbatt sensor)
const float VREF     = 3.3f;   // ESP32 ADC reference
const float ADC_MAX  = 4095.0f;

// ------- battery model -------
const float C_nom_Ah = 2.0f;
float SoC      = 100.0f;
float Qused_Ah = 0.0f;
float I_prev   = NAN;
unsigned long t_prev_ms = 0;

// ------- loop timing -------
const unsigned long LOOP_MS = 1000;

/* ------------------------------------------------------------------
   Helper: read pin → volts
------------------------------------------------------------------ */
inline float readVolts(int pin)
{
  return analogRead(pin) / ADC_MAX * VREF;
}

/* ------------------------------------------------------------------
   Draw battery icon on u8g2 display (x,y = top-left)
------------------------------------------------------------------ */
void drawBatteryIcon(int x, int y, int w, int h, int filledBars)
{
  const int tipW = w / 8;                       // little positive terminal

  /* ----- decide bar & gap widths so: 5*bar + 6*gap = w ----- */
  int gap = 2;                                  // try 2-px gaps first
  int bar = (w - 6*gap) / 5;                    // bar width that fits
  if (bar < 1) {                                // icon too small? fall back
    gap = 1;
    bar = (w - 6*gap) / 5;
  }

  /* ----- outline & tip ----- */
  display.drawFrame(x, y, w, h);                // outer rectangle
  display.drawBox  (x + w, y + h/4, tipW, h/2); // terminal box

  /* ----- draw bars ----- */
  int barH = h - 2*gap;
  int bx = x + gap;                             // first bar left edge
  for (int i = 0; i < filledBars; ++i) {
    display.drawBox(bx, y + gap, bar, barH);
    bx += bar + gap;                            // next bar start
  }
}

/* ------------------------------------------------------------------ */
void setup()
{
  Serial.begin(9600);

  // I²C on default DevKit pins (SDA 21, SCL 22)
  Wire.begin(21,22);
  display.begin();
  display.setFont(u8g2_font_6x12_tr);

  t_prev_ms = millis();
}

/* ------------------------------------------------------------------ */
void loop()
{
  /* ---- time step & trapezoid dt ---- */
  unsigned long now_ms = millis();
  float dt_s  = (now_ms - t_prev_ms) / 1000.0f;
  t_prev_ms   = now_ms;
  float dt_h  = dt_s / 3600.0f;

  /* ---- sensor voltages ---- */
  float Vo    = readVolts(PIN_VO);
  float Vbatt = readVolts(PIN_VBAT);

  /* ---- currents ---- */
  float I_logic = ((Vo    - 0.9f)  / 510.0f) / 0.01f;
  float I_motor = ((Vbatt - 1.27f) / 100.0f) / 0.01f;
  I_logic = max(0.0f, I_logic);
  I_motor = max(0.0f, I_motor);
  float I_now = I_logic + I_motor;

  /* ---- trapezoidal charge integration ---- */
  if (!isnan(I_prev))
    Qused_Ah += 0.5f * (I_prev + I_now) * dt_h;
  I_prev = I_now;

  SoC = constrain(100.0f - (Qused_Ah / C_nom_Ah * 100.0f), 0.0f, 100.0f);

  /* ---- total power ---- */
  float P_total = Vo * I_logic + Vbatt * I_motor;

  /* ============ DRAW ============ */
  display.clearBuffer();

  // battery icon top-left (width¼ screen)
  const int iconW = 48, iconH = 24;
  drawBatteryIcon(4, 4, iconW, iconH, min(5,int(SoC)/20));

  // % text just under icon
  display.setFont(u8g2_font_ncenB14_tr);
  char socBuf[8];
  snprintf(socBuf, sizeof(socBuf), "%d%%", int(SoC+0.5f));
  int16_t tw = display.getStrWidth(socBuf);
  display.drawStr((iconW + 8 - tw)/2 + 4, iconH + 28, socBuf);

  // power text right half
  display.setFont(u8g2_font_6x12_tr);
  char pBuf[14];
  snprintf(pBuf, sizeof(pBuf), "P = %.1f W", P_total);
  int16_t px = 64 + (64 - display.getStrWidth(pBuf))/2;
  display.drawStr(px, 25, pBuf);

  // small Vo / Vbatt readout (optional)
  char vBuf[16];
  snprintf(vBuf, sizeof(vBuf), "Vo=%.2fV", Vo);
  display.drawStr(px, 42, vBuf);
  snprintf(vBuf, sizeof(vBuf), "Vb=%.2fV", Vbatt);
  display.drawStr(px, 54, vBuf);

  display.sendBuffer();

  delay(max(10UL, LOOP_MS - (millis() - now_ms)));  // maintain ~1 s loop
}
