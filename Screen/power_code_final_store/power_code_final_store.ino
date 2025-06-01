/******************************************************************
  BatteryMonitor_SH1106_U8g2.ino
  ---------------------------------------------------------------
  ‣ ESP32-DevKit (on-chip ADC only)
       Vlogic sensor  → GPIO39  (ADC1_CH3)
       Vmotor sensor  → GPIO34  (ADC1_CH6)
  ‣ 1.3" 128×64 OLED with SH1106 controller
       VCC → 3V3   |  GND → GND
       SCK → GPIO22|  SDA → GPIO21
  ‣ Ilogic = ((Vlogic  - 0.9)  / 510) / 0.01
    Imotor = ((Vmotor - 1.27) / 100) / 0.01
  ‣ Charge integrated with trapezoidal rule
  ‣ Screen layout
       ┌─────────────┐
       │  BAT ICON    │
       │  + % text    │
       │──────────────│
       │   P = XX.X W │
       │              │
       └──────────────┘

  Additionally prints Ilogic and Imotor to Serial Monitor each loop.
******************************************************************/

#include <Wire.h>
#include <U8g2lib.h>

// ------- OLED (SH1106) -------
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// ------- ADC pins & constants -------
const int   PIN_VLOGIC = 39;   // GPIO39 (your Vlogic sensor)
const int   PIN_VMOTOR = 34;   // GPIO34 (your Vmotor sensor)
const float VREF       = 3.3f; // ESP32 ADC reference
const float ADC_MAX    = 4095.0f;

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
void drawBatteryIcon(int x, int y, int w, int h, int barsFilled)
{
  const int tipW = w / 8;

  display.drawFrame(x, y, w, h);           // outline
  display.drawBox(x + w, y + h/4, tipW, h/2); // tip

  // calculate even bar/gap widths
  int gap = 2;
  int bar = (w - 6*gap) / 5;
  if (bar < 1) { gap = 1; bar = (w - 6*gap) / 5; }
  int barH = h - 2*gap;
  int bx = x + gap;
  for (int i = 0; i < barsFilled; ++i) {
    display.drawBox(bx, y + gap, bar, barH);
    bx += bar + gap;
  }
}


/* ------------------------------------------------------------------ */
void setup()
{
  Serial.begin(9600);

  // I²C on default DevKit pins (SDA 21, SCL 22)
  Wire.begin(21, 22);
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
  float Vlogic = readVolts(PIN_VLOGIC);
  float Vmotor = readVolts(PIN_VMOTOR);

  /* ---- currents ---- */
  float Ilogic = ((Vlogic - 0.9f) / 510.0f) / 0.01f;
  float Imotor = ((Vmotor - 1.27f) / 100.0f) / 0.01f;
  Ilogic = max(0.0f, Ilogic);
  Imotor = max(0.0f, Imotor);
  float I_now = Ilogic + Imotor;

  /* ---- trapezoidal charge integration ---- */
  if (!isnan(I_prev)) {
    Qused_Ah += 0.5f * (I_prev + I_now) * dt_h;
  }
  I_prev = I_now;

  SoC = constrain(100.0f - (Qused_Ah / C_nom_Ah * 100.0f), 0.0f, 100.0f);

  /* ---- total power ---- */
  float P_total = Vlogic * Ilogic + Vmotor * Imotor;

  /* ---- print currents to terminal ---- */
  Serial.print("Ilogic: ");
  Serial.print(Ilogic, 3);
  Serial.print(" A    Imotor: ");
  Serial.print(Imotor, 3);
  Serial.println(" A");

  /* ============ DRAW ============ */
  display.clearBuffer();

  // battery icon top-left (width¼ screen)
  const int iconW = 48, iconH = 24;
  drawBatteryIcon(4, 4, iconW, iconH, min(5, int(SoC) / 20));

  // % text just under icon
  display.setFont(u8g2_font_ncenB14_tr);
  char socBuf[8];
  snprintf(socBuf, sizeof(socBuf), "%d%%", int(SoC + 0.5f));
  int16_t tw = display.getStrWidth(socBuf);
  display.drawStr((iconW + 8 - tw) / 2 + 4, iconH + 28, socBuf);

  // power text right half
  display.setFont(u8g2_font_6x12_tr);
  char pBuf[14];
  snprintf(pBuf, sizeof(pBuf), "P = %.1f W", P_total);
  int16_t px = 64 + (64 - display.getStrWidth(pBuf)) / 2;
  display.drawStr(px, 25, pBuf);

  // small Vlogic / Vmotor readout (optional)
  char vBuf[16];
  snprintf(vBuf, sizeof(vBuf), "Vl=%.2fV", Vlogic);
  display.drawStr(px, 42, vBuf);
  snprintf(vBuf, sizeof(vBuf), "Vm=%.2fV", Vmotor);
  display.drawStr(px, 54, vBuf);

  display.sendBuffer();

  delay(max(10UL, LOOP_MS - (millis() - now_ms)));  // maintain ~1 s loop
}
