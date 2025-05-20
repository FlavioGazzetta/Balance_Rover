#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// — simulate a current reading (replace with your sensor code) —
float readCurrent_mA() {
  return analogRead(A0) / 4095.0 * 500.0;  // 0–500 mA range
}

void setup() {
  Wire.begin(21, 22);                      // ESP32 I2C: SDA, SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  float current_mA = readCurrent_mA();

  // determine how many of the 4 bars to fill
  int bars = 0;
  if (current_mA > 100) bars = 1;
  if (current_mA > 200) bars = 2;
  if (current_mA > 300) bars = 3;
  if (current_mA > 400) bars = 4;

  display.clearDisplay();

  // --- BATTERY ICON (top half) ---
  const int batW    = 60;              // battery body width
  const int batH    = 26;              // thinner height
  const int tipW    = 8;               
  const int tipH    = batH / 2;        // tip is half the body height
  const int batX    = (SCREEN_WIDTH - (batW + tipW)) / 2;  // center horizontally
  const int batY    = 4;               // top margin

  // outline
  display.drawRect(batX, batY, batW, batH, SSD1306_WHITE);
  // tip
  display.fillRect(batX + batW,batY + (batH - tipH)/2,tipW, tipH,SSD1306_WHITE);

  // fill bars inside
  const int slotCount = 4;
  const int slotW     = 14;
  const int slotH     = batH - 2*4;    // leave 4px padding top & bottom
  const int slotXoff  = 4;             // left padding
  const int slotYoff  = 4;             // top padding
  const int slotGap   = 18;            // distance between bar origins

  for (int i = 0; i < bars; i++) {
    int x = batX + slotXoff + i * slotGap;
    int y = batY + slotYoff;
    display.fillRect(x, y, slotW, slotH, SSD1306_WHITE);
  }

  // --- CURRENT DISPLAY (bottom half) ---
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int txtY = SCREEN_HEIGHT/2 + 4;      // start just into bottom half
  display.setCursor(30, txtY);
  display.print(current_mA, 1);
  display.print(" mA");

  display.display();
  delay(1000);
}
