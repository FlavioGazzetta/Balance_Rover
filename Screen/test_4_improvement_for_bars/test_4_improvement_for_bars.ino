#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// — cycle through 0,100,200…500 mA so you can watch 0→5 bars fill —
float simulateBatteryFill() {
  static int step = 0;
  step = (step + 1) % 6;       // cycles 0–5
  return step * 100.0;         // yields 0,100,200,300,400,500 mA
}

float readCurrent_mA() {
  return analogRead(A0) / 4095.0 * 500.0;  // 0–500 mA range
}

void setup() {
  Wire.begin(21, 22);  // ESP32 I2C: SDA = 21, SCL = 22
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  // 1) get simulated current
  float current_mA = simulateBatteryFill();

  // 2) compute bars: 0–5, with 0 for 0–99, 1 for 100–199, etc.
  int bars = int(current_mA) / 100;
  if (bars > 5) bars = 5;

  display.clearDisplay();

  // --- BATTERY ICON (top half) ---
  const int slotCount  = 5;    // five bars
  const int barSpacing = 2;    // px between bars and edges
  const int slotW      = 9;    // bar width in px
  const int barMargin  = barSpacing;
  const int batW       = slotCount * slotW + (slotCount + 1) * barSpacing;  // 5*9 + 6*2 = 57
  const int batH       = 26;
  const int tipW       = 8;
  const int tipH       = batH / 2;
  const int batX       = (SCREEN_WIDTH - (batW + tipW)) / 2;
  const int batY       = 4;

  // outline & tip
  display.drawRect(batX, batY, batW, batH, SSD1306_WHITE);
  display.fillRect(
    batX + batW,
    batY + (batH - tipH) / 2,
    tipW, tipH,
    SSD1306_WHITE
  );

  // compute bar drawing
  const int slotH = batH - 2 * barMargin;  
  const int slotY = batY + barMargin;
  const int slotX0 = batX + barMargin;

  // draw filled bars
  for (int i = 0; i < bars; i++) {
    int x = slotX0 + i * (slotW + barSpacing);
    display.fillRect(x, slotY, slotW, slotH, SSD1306_WHITE);
  }

  // --- CURRENT DISPLAY (bottom half) ---
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int txtY = SCREEN_HEIGHT / 2 + 4;
  display.setCursor((SCREEN_WIDTH - 6 * 12) / 2, txtY);  // center approx for "500 mA"
  display.print(current_mA, 1);
  display.print(" mA");

  display.display();
  delay(1000);
}
