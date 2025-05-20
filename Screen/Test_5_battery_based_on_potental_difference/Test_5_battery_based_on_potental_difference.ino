#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64

// actual ESP32 ADC-capable pins
#define PIN_VOLTAGE    36    // ADC1_CH0
#define PIN_CURRENT    39    // ADC1_CH3

#define VREF           3.3
#define ADC_MAX        4095.0

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float readVoltage() {
  return analogRead(PIN_VOLTAGE) / ADC_MAX * VREF;
}

float readCurrent_mA() {
  return analogRead(PIN_CURRENT) / ADC_MAX * 500.0;
}

void setup() {
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  //--- read sensors
  float voltage    = readVoltage();
  float current_mA = readCurrent_mA();
  float current_A  = current_mA / 1000.0;

  //--- compute % & bars, clamp <5% to 0
  float pct = voltage / VREF * 100.0;
  if (pct < 5.0) pct = 0.0;
  pct = constrain(pct, 0.0, 100.0);
  int bars = int(pct) / 20;       // 0–5 bars
  bars = min(bars, 5);

  //--- clear & draw quadrant lines
  display.clearDisplay();
  int midX = SCREEN_WIDTH  / 2;
  int midY = SCREEN_HEIGHT / 2;
  display.drawLine(midX, 0,    midX, SCREEN_HEIGHT, SSD1306_WHITE);
  display.drawLine(0,    midY, SCREEN_WIDTH, midY,    SSD1306_WHITE);

  //--- top-left: little battery @75% scale, centered in (0,0)-(midX,midY)
  const int slotCount  = 5;
  const int slotW      = 9;
  const int barSpacing = 2;
  const int tipW       = 8;
  const int batH       = 26;
  // 75% scale:
  const int small_slotW      = (slotW      * 3) / 4;  // ~6
  const int small_barSpacing = (barSpacing * 3) / 4;  // ~1
  const int small_tipW       = (tipW      * 3) / 4;  // ~6
  const int small_batH       = (batH      * 3) / 4;  // ~19
  const int small_barMargin  = small_barSpacing;

  int small_batW = slotCount * small_slotW + (slotCount + 1) * small_barSpacing;
  // center it:
  int q1_cx = midX / 2;
  int q1_cy = midY / 2;
  int small_batX = q1_cx - small_batW/2;
  int small_batY = q1_cy - small_batH/2;

  // draw outline + tip
  display.drawRect(small_batX, small_batY, small_batW, small_batH, SSD1306_WHITE);
  display.fillRect(
    small_batX + small_batW,
    small_batY + (small_batH - small_batH/2)/2,
    small_tipW, small_batH/2,
    SSD1306_WHITE
  );

  // fill bars
  int slotY  = small_batY + small_barMargin;
  int slotX0 = small_batX + small_barMargin;
  int slotH  = small_batH - 2 * small_barMargin;
  for (int i = 0; i < bars; i++) {
    int x = slotX0 + i * (small_slotW + small_barSpacing);
    display.fillRect(x, slotY, small_slotW, slotH, SSD1306_WHITE);
  }

  //--- bottom-left: % textSize(2), centered in (0,midY)-(midX,SCREEN_HEIGHT)
  String pctStr = String(int(pct)) + "%";
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(pctStr, 0, 0, &x1, &y1, &w, &h);
  int pctX = (midX  - w)/2;
  int pctY = midY + (midY - h)/2;
  display.setCursor(pctX, pctY);
  display.print(pctStr);

  //--- top-right: current in A, textSize(1), centered in (midX,0)-(SCREEN_WIDTH,midY)
  String curStr = String(current_A, 2) + " A";
  display.setTextSize(1);
  display.getTextBounds(curStr, 0, 0, &x1, &y1, &w, &h);
  int curX = midX + (midX - w)/2;
  int curY = (midY - h)/2;
  display.setCursor(curX, curY);
  display.print(curStr);

  //--- bottom-right: smiley, centered in (midX,midY)-(SCREEN_WIDTH,SCREEN_HEIGHT)
  int q4_cx = midX + midX/2;
  int q4_cy = midY + midY/2;
  int r = min(midX, midY)/2 - 5;
  display.drawCircle(q4_cx, q4_cy, r, SSD1306_WHITE);
  display.fillCircle(q4_cx - r/2, q4_cy - r/3, 2, SSD1306_WHITE);
  display.fillCircle(q4_cx + r/2, q4_cy - r/3, 2, SSD1306_WHITE);
  display.drawCircleHelper(q4_cx, q4_cy + 2, r - 4, 4, SSD1306_WHITE);
  display.drawCircleHelper(q4_cx, q4_cy + 2, r - 4, 8, SSD1306_WHITE);

  //--- update
  display.display();
  delay(1000);
}
