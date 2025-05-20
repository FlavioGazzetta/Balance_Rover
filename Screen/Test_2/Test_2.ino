#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Wire.begin(21, 22); // SDA, SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
}

void loop() {
  // Simulated current in mA (replace with real sensor value)
  float current_mA = analogRead(A0) / 4095.0 * 500; // fake range 0–500mA

  // Calculate number of bars (0–4)
  int bars = 0;
  if (current_mA > 100) bars = 1;
  if (current_mA > 200) bars = 2;
  if (current_mA > 300) bars = 3;
  if (current_mA > 400) bars = 4;

  display.clearDisplay();

  // Draw battery outline
  int x = 10, y = 10;
  display.drawRect(x, y, 40, 20, SSD1306_WHITE); // Body
  display.fillRect(x + 40, y + 6, 4, 8, SSD1306_WHITE); // Tip

  // Draw bars
  for (int i = 0; i < bars; i++) {
    display.fillRect(x + 3 + i * 9, y + 3, 7, 14, SSD1306_WHITE);
  }

  // Show simulated current value
  display.setCursor(60, 15);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(current_mA, 0);
  display.print(" mA");

  display.display();
  delay(1000);
}
