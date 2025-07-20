# Balance Robot Project

**Electronics Design Project 2**

🔗 [GitHub Repository](https://github.com/FlavioGazzetta/Balance_Rover.git)

---

## Authors

- Evangelia (Lia) Kommata (lk823)  
- Flavio Gazzetta (fg723)  
- Nabiha Saqib (ns3323)  
- Shenghong Liu (sl4223)  
- Zecheng Zhu (zz4723)  

---

## Table of Contents

- [Abstract](#abstract)  
- [Product Design](#product-design)  
- [Project Management](#project-management)  
- [Requirements Capture](#requirements-capture)  
- [Modelling](#modelling)  
- [Control System](#control-system)  
- [Manual Remote Control](#manual-remote-control)  
- [Head Unit](#head-unit)  
- [Battery Analysis](#battery-analysis)  
- [User Interface](#user-interface)  
  - [Mobile App](#mobile-app)  
  - [Weather UI Mode](#weather-ui-mode)  
  - [AI Chat Mode](#ai-chat-mode)  
  - [Person Following Mode](#person-following-mode)  
  - [Controller Communication](#controller-communication)  
- [Evaluation](#evaluation)  
  - [Finances](#finances)  
  - [Review of Product](#review-of-product)  
  - [Future Work](#future-work)  
- [Acknowledgements](#acknowledgements)  

---

## Abstract

Campus exploration is enhanced by a two‑wheel self‑balancing “Campus Tour Guide” robot that:

- Maintains stability with dual PID loops + complementary filter  
- Follows a person via YOLOv8-nano & UDP  
- Offers manual control via a Flutter app + ESP32 REST API  
- Displays battery health & weather on OLED + mobile UI  
- Provides voice Q&A with OpenAI Chat Completion API  
- Is built modularly with safety, maintainability & sustainability in mind  

---

## Product Design

- **Chassis & Layout**: Low CG, stacked ESP32/RPi modules, Velcro/3D‑printed mounts  
- **Mechanical**: 3D‑printed head, reinforced high‑impact zones  
- **Power**: Isolated 5 V logic & 12 V motor rails, decoupling, ground plane  
- **Interaction**: Handle, 5‑bar battery indicator, menu UI, audio output  
- **Safety**: Fuses, tilt‑resistant wheelbase, easy‑swap terminals  
- **Sustainability**: PLA prints, reusable/upgradable modules  

---

## Project Management

- **Framework**: Agile 1‑week sprints, frequent demos & reviews  
- **Roles**: Fair task allocation, cross‑module integration by all members  
- **Timeline**: Gantt chart maintained; weekly online (Fri) & in‑person (Tue) meetings  
- **Risks**: IMU noise, motor overheating, loop overload, part delays, tipping  

---

## Requirements Capture

### Core Requirements

| Requirement                        | Adaptation & Benchmark                                     |
|------------------------------------|------------------------------------------------------------|
| **Autonomous person‑following**    | YOLOv8-nano over UDP; ≥ 90 % accuracy over 5 m run         |
| **Two‑wheel balance**              | Dual PID + complementary filter; recover 10° tilt ≤ 5 s     |
| **Manual remote control**          | Flutter app ↔ ESP32 REST; round‑trip < 100 ms @ 5 m         |
| **Power status display**           | OLED + app; SoC error ≤ 5 %, update ≥ 1 Hz                 |
| **UI mode‑switch**                 | < 200 ms switch; ≥ 30 Hz refresh                           |
| **3D‑printed head unit**           | 15° camera tilt; ± 30° FOV; ≥ 2 kg handle lift             |

### Stability & Recovery

- **Stationary**: ≥ 5 min upright without support  
- **Straight‑line**: 0.5 m/s for 2 min  
- **Turning**: ≤ 60°/s for full battery life  
- **Disturbance**: Regain balance ≤ 5 s after push  

---

## Modelling

- **Dynamics**: Two‑wheeled inverted pendulum + stepper torque model  
- **Linearization**: Small‑angle approximation → state‑space form  
- **Control**: From MPC concept to LQR → robust real‑time PID  

---

## Control System

- **Inner Loop (20 ms)**: Dual PIDs for tilt & rotation  
- **Outer Loop (100 ms)**: Position via tilt set‑point  
- **Filter**: Complementary filter (c = 0.98) fuses accel & gyro  
- **Tuning**: Simulation (MuJoCo), trial‑and‑error, loop‑shaping considered  

---

## Manual Remote Control

- **2D Joystick UI**: Maps (x, y) ∈ [–1,1] to tilt/heading set‑points  
- **REST API**: ESP32 handles `/set_mode` & joystick JSON every 100 ms  
- **Smooth Control**: Inner PID blends biases into wheel speeds  

---

## Head Unit

- **Design**: Handle + secure camera mount with 15° tilt  
- **Material**: PLA, 50 % honeycomb infill for strength  
- **Iterations**: Three prototypes → final CAD + printed version  

---

## Battery Analysis

- **Coulomb Counting**: Trapezoidal integration, calibrated zero offset  
- **Lookup Table**: Discharge curve for 12‑cell pack → SoC lookup  
- **Hybrid**: Use voltage curve at startup, then Coulomb count  
- **Current Sense**: Differential amp (TLV272I), gains 510× (logic) & 100× (motor)  
- **Display**: OLED & app show SoC %, instantaneous power (5 s avg)  

---

## User Interface

### Mobile App

- Flutter (iOS/Android) with four modes  
- Uses phone hotspot to connect to ESP32 & Raspberry Pi  

### Weather UI Mode

- **Sensor**: GY‑39 (temp, humidity, pressure, lux) + GPS module  
- **API**: Google Weather by device coords; fallback default location if GPS fails  
- **Display**: Robot vs. phone data in separate cards  

### AI Chat Mode

- **Audio I/O**: Phone STT → server memory → OpenAI → MP3 via ESP32 & SD card  
- **RAG**: Summarised chat memory & context from Django REST + PostgreSQL  
- **Auth**: JWT secured; ReactJS admin UI for logs & GPS  

### Person Following Mode

- **Streaming**: Pi → Laptop (JPEG @ 1/15 s) → ZeroMQ → YOLOv8-nano → JSON → Pi → UDP → ESP32  
- **Tracking**: SORT for consistent IDs; fallback to largest bounding box  
- **Control**: xCam → yaw set‑point; areaCam → position set‑point  

### Controller Communication

- **Protocol**: ESP‑NOW P2P (≤ 1 ms latency)  
- **Rejected**: Wi‑Fi (10–100 ms), BLE (unpredictable GATT delays)  
- **Data**: `deltaYaw`, `deltaPitch` packets  

---

## Evaluation

### Finances

| Component                   | Qty | Unit (£) | Total (£) | Used?      |
|-----------------------------|-----|----------|-----------|------------|
| Breadboards (MB‑102)        | 3   | 6.37     | 19.11     | Most       |
| TLV272I Op‑Amps             | 6   | 0.89     | 5.34      | Most       |
| Electret Mic Module         | 1   | 0.76     | 0.76      | No         |
| Mic & Audio Amp             | 1   | 4.55     | 4.55      | Yes        |
| PAM8403 Amp                 | 1   | 3.19     | 3.19      | Yes        |
| Speaker                     | 1   | 1.55     | 1.55      | Yes        |
| **Total**                   |     |          | **34.50** |            |
| **Remaining Budget (£60)**  |     |          | **25.50** |            |

### Review of Product

- **Balance**: Met/exceeded all stability tests  
- **Control**: Tilt recovery 2 s (vs ≤ 5 s), latency 85 ms (vs < 100 ms)  
- **Following**: 92 % accuracy, 48 ms latency (vs ≤ 50 ms)  
- **UI & Power**: Mode‑switch 180 ms, SoC error ≤ 4 %  
- **Mechanical**: FOV ± 32° yaw  

### Future Work

- **Energy**: Regenerative braking, dynamic CPU scaling  
- **Navigation**: SLAM + LiDAR, obstacle detection  
- **Head Unit**: Quick‑adjust tilt mechanism  
- **UX**: User studies, multi‑language, voice/haptic feedback  
- **Team**: Daily stand‑ups, cross‑training  

---