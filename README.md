# Balance-Bot Project Overview

This organization contains five repositories that implement the key modules of our two-wheeled self-balancing tour guide robot. They are intended to work together to simulate, control, track, and display status for the rover.

---

## 1. Rover_Controller

**Description:**  
Firmware for the ESP32 balance-bot, implementing:
- Inner‐loop PID (20 ms) for tilt stabilization  
- Outer‐loop P controller (200 ms) for position following  
- MPU6050 tilt sensing + complementary filter  
- UDP follower mode: receives `x_center` & `area` from the RPi → computes heading & distance set-points  
- Manual‐drive mode via Wi-Fi UI buttons (↑ ↓ ← →)  
- Spin‐in-place & auto-sync straight-line modes  

**Key files:**
- `main.cpp` – setup, main control loops, UDP parsing & PID  
- `step.h` / `step.cpp` – stepper driver  
- `mpu6050.cpp` – sensor filtering  
- `WiFiUI.cpp` – REST API & button handlers  

---

## 2. person_tracking

**Description:**  
Distributed person-following client/server using YOLOv8-nano:  
- Raspberry Pi client (`pi_client.py`):  
  - Captures 320×240 JPEG thumbnails @15 fps  
  - PUSH → laptop via ZeroMQ (tcp://…:5555)  
  - SUBscribes to bounding-box JSON from server (tcp://…:5556)  
  - Rescales, selects box, sends `x_center area` → ESP32 via UDP  
- GPU server (`gpu_server.py`):  
  - PULL with HWM=1 (drop old frames)  
  - Decode JPEG, inference on GPU (FP16)  
  - SORT‐based tracker → persistent IDs  
  - PUB bounding boxes + `track_id` JSON  

**Key files:**
- `pi_client.py`  
- `gpu_server.py`    

---

## 3. Controller_Python_SIM

**Description:**  
Python + MuJoCo simulation of the two-wheeled inverted-pendulum rover:  
- Continuous‐time stepper model  
- Complementary filter + PID loops  
- Real-time GUI (Tkinter) to tweak gains & visualize θ, x  

**Key files:**
- `sim_rover.py` – MuJoCo environment & control  
- `controller.py` – PID/complementary filter implementation  
- `gui.py` – live plots & parameter sliders  

---

## 4. Screen

**Description:**  
OLED battery & power‐status display firmware (ESP32):  
- Differential‐amplifier current sensing (TLV272)  
- Coulomb‐count + lookup‐table for SoC  
- Voltage‐divider ADC for pack voltage  
- 128×64 SSD1306 OLED UI showing:  
  - State-of-Charge (%)  
  - Instantaneous power (W)  

**Key files:**
- `screen.ino` – Arduino sketch for ADC, filtering, display  
- `lookup_table.h` – discharge‐curve data  

---

## 5. ESPtoESP

**Description:**  
Proof-of-concept for ESP-NOW peer-to-peer messaging:  
- Two ESP32 modules exchanging control & telemetry frames  
- Minimal-overhead binary packets  
- Demonstrates sub-millisecond hops without Wi-Fi AP  

**Key files:**
- `sender.ino` – broadcasts control commands  
- `receiver.ino` – prints telemetry, toggles LED  

---

### Getting Started

1. **Clone** the repositories you need.  
2. **Install** dependencies (MuJoCo, Python packages, Arduino libs).  
3. **Flash** the appropriate ESP32 binary for `Rover_Controller`, `Screen`, or `ESPtoESP`.  
4. **Run** `gpu_server.py` on your CUDA-equipped PC, then `pi_client.py` on the Pi.  
5. **Launch** the MuJoCo sim via `sim_rover.py` to tune gains offline.

---