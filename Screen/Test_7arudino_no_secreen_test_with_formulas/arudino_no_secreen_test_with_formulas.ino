/*
  BatteryMonitor_Sim_Human_9600.ino
  ------------------------------------------------
  - ESP32-WROOM-32D only
  - 9600 baud
  - Always simulates Vo & Vbatt
  - Prints each value on its own line, with a blank line between seconds
*/

#include <Arduino.h>
#include <math.h>

// Sensor offsets & sensitivities (for simulation math)
const float VofsL = 2.5, kL = 0.01;
const float VofsM = 2.5, kM = 0.01;

// Battery capacity (Ah)
const float C_nom = 2.2;

// State variables
float SoC      = 100.0;
float Qused_Ah = 0.0;

const unsigned long LOOP_MS = 1000;  // 1 second

// —— Simulation only ——
float t_sim = 0.0;
float readVo() {
  // logic current oscillates around 0.2 A
  float I = 0.2 + 0.1 * sin(2 * PI * t_sim / 60.0);
  t_sim += 1.0;
  return VofsL + kL * I;
}
float readVbatt() {
  // motor current oscillates around 0.5 A
  float I = 0.5 + 0.2 * sin(2 * PI * t_sim / 30.0);
  return VofsM + kM * I;
}

void setup() {
  Serial.begin(9600);
  // no while(!Serial) on ESP32
  Serial.println("Battery Monitor (Simulation Only) Started");
  Serial.println();  // blank line before first block
}

void loop() {
  static unsigned long sec = 0;

  // 1) Simulate voltages
  float Vo    = readVo();
  float Vbatt = readVbatt();

  // 2) Compute currents (A) and clamp negatives
  float I_l = (Vo    - VofsL) / kL;
  float I_m = (Vbatt - VofsM) / kM;
  if (I_l < 0) I_l = 0;
  if (I_m < 0) I_m = 0;

  // 3) Compute powers (W)
  float P_l = Vo    * I_l;
  float P_m = Vbatt * I_m;
  float P_t = P_l + P_m;

  // 4) Integrate charge used (Ah)
  float dt_h = (LOOP_MS / 1000.0) / 3600.0;
  Qused_Ah  += (I_l + I_m) * dt_h;

  // 5) Update SoC (%)
  SoC = 100.0 - (Qused_Ah / C_nom * 100.0);
  SoC = constrain(SoC, 0.0, 100.0);

  // 6) Print human-readable block
  Serial.print("Time    : "); Serial.print(sec);    Serial.println(" s");
  Serial.print("Vo      : "); Serial.print(Vo, 3);  Serial.println(" V");
  Serial.print("Vbatt   : "); Serial.print(Vbatt, 3);Serial.println(" V");
  Serial.print("I_logic : "); Serial.print(I_l, 3); Serial.println(" A");
  Serial.print("I_motor : "); Serial.print(I_m, 3); Serial.println(" A");
  Serial.print("P_logic : "); Serial.print(P_l, 2); Serial.println(" W");
  Serial.print("P_motor : "); Serial.print(P_m, 2); Serial.println(" W");
  Serial.print("P_total : "); Serial.print(P_t, 2); Serial.println(" W");
  Serial.print("SoC     : "); Serial.print(SoC, 2);   Serial.println(" %");
  Serial.println();  // blank line before next second

  sec++;
  delay(LOOP_MS);
}
