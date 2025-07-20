Balance Robot Project
Electronics Design Project 2
https://github.com/FlavioGazzetta/Balance_Rover.git
Authors
Evangelia (Lia) Kommata (lk823), Flavio Gazzetta (fg723), Nabiha Saqib (ns3323),
Shenghong Liu (sl4223), Zecheng Zhu (zz4723)
Word count: 9870
July 20, 2025
Contents
1
Introduction
4
1.1
Abstract . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
4
1.2
Background . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
5
1.3
Product Design . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
5
1.3.1
Chassis and Structural Layout . . . . . . . . . . . . . . . . . . . .
5
1.3.2
Mechanical Modifications . . . . . . . . . . . . . . . . . . . . . . .
5
1.3.3
Power Management Layout
. . . . . . . . . . . . . . . . . . . . .
5
1.3.4
Human and Robot Interaction . . . . . . . . . . . . . . . . . . . .
5
1.3.5
Design for Safety and Maintenance . . . . . . . . . . . . . . . . .
6
1.3.6
Sustainability Considerations
. . . . . . . . . . . . . . . . . . . .
6
1.4
Project Management . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
7
1.4.1
Project Framework Adapted . . . . . . . . . . . . . . . . . . . . .
7
1.4.2
Role Allocation . . . . . . . . . . . . . . . . . . . . . . . . . . . .
7
1.4.3
Timeline . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
8
1.4.4
Weekly Plan . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
8
1.4.5
Risk and Contingencies . . . . . . . . . . . . . . . . . . . . . . . .
8
2
Requirements Capture
10
2.1
Core Requirements, Adaptations, and Verification Benchmarks . . . . . .
10
2.2
Stability and Recovery Requirements . . . . . . . . . . . . . . . . . . . .
10
2.3
Manual Remote Control Requirement . . . . . . . . . . . . . . . . . . . .
11
2.4
Extensible Module Architecture . . . . . . . . . . . . . . . . . . . . . . .
11
2.5
Non-Technical Requirements . . . . . . . . . . . . . . . . . . . . . . . . .
12
3
Modelling
14
3.0.1
Stepper Motor Model . . . . . . . . . . . . . . . . . . . . . . . . .
14
3.0.2
Dynamic model for a two wheeled inverted pendulum . . . . . . .
14
3.1
Model Predictive Control Approach . . . . . . . . . . . . . . . . . . . . .
16
4
Control System
18
4.1
Design Overview
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
18
4.2
Sensors and Signal Processing . . . . . . . . . . . . . . . . . . . . . . . .
18
4.3
Inner Loop PID controller . . . . . . . . . . . . . . . . . . . . . . . . . .
19
4.3.1
Static PID . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
20
4.3.2
Rotation PID . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
20
4.4
Outer Loop controller . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
21
4.5
Parameter Tuning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
25
4.5.1
Simulation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
25
4.5.2
Trial-and-Error Tuning . . . . . . . . . . . . . . . . . . . . . . . .
25
4.5.3
Loop-Shaping Control Design (Considered) . . . . . . . . . . . . .
26
5
Manual Remote Control
28
6
Head Unit
29
6.1
Utility . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
29
6.2
Design Process
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
29
6.2.1
First Design . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
1
6.2.2
Second Design . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
30
6.2.3
Third Design
. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
31
7
Battery Analysis
33
7.1
Coulomb Counting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
33
7.2
Discharge Curve Method . . . . . . . . . . . . . . . . . . . . . . . . . . .
34
7.3
Evaluation of Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . .
35
7.4
Power Consumption
. . . . . . . . . . . . . . . . . . . . . . . . . . . . .
36
7.5
Implementation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
36
7.5.1
Current Sensing Circuit
. . . . . . . . . . . . . . . . . . . . . . .
36
7.5.2
Sensing Circuit & Calibration . . . . . . . . . . . . . . . . . . . .
38
7.5.3
Voltage Measurement . . . . . . . . . . . . . . . . . . . . . . . . .
39
7.5.4
Display
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
39
8
User Interface
41
8.1
Cross Platform Mobile App
. . . . . . . . . . . . . . . . . . . . . . . . .
41
8.1.1
Mode Switching . . . . . . . . . . . . . . . . . . . . . . . . . . . .
42
8.1.2
On-device Capabilities . . . . . . . . . . . . . . . . . . . . . . . .
42
8.2
AWS EC2 Server . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
42
8.2.1
JWT Authentication . . . . . . . . . . . . . . . . . . . . . . . . .
42
8.2.2
Database
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
43
8.2.3
Admin Website . . . . . . . . . . . . . . . . . . . . . . . . . . . .
43
8.3
Weather UI Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
44
8.3.1
Weather Sensor . . . . . . . . . . . . . . . . . . . . . . . . . . . .
44
8.3.2
GPS Coordinate Processing . . . . . . . . . . . . . . . . . . . . .
44
8.3.3
Google Weather API . . . . . . . . . . . . . . . . . . . . . . . . .
45
8.3.4
GPS Signal Contingency . . . . . . . . . . . . . . . . . . . . . . .
45
8.3.5
Display
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
45
8.4
AI Chat Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
45
8.4.1
Prompting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
45
8.4.2
Response
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
46
8.4.3
Chat Memory . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
46
8.4.4
Retrieval-Augmented Generation (RAG) . . . . . . . . . . . . . .
47
8.5
Person Following Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . .
48
8.5.1
YOLO Algorithm . . . . . . . . . . . . . . . . . . . . . . . . . . .
49
8.5.2
Server Setup (CUDA)
. . . . . . . . . . . . . . . . . . . . . . . .
51
8.5.3
Streaming Architecture . . . . . . . . . . . . . . . . . . . . . . . .
51
8.5.4
Designated Person Selection . . . . . . . . . . . . . . . . . . . . .
51
8.5.5
ESP32 Person following integration . . . . . . . . . . . . . . . . .
53
8.5.6
Custom Model
. . . . . . . . . . . . . . . . . . . . . . . . . . . .
53
8.6
Controller Communication . . . . . . . . . . . . . . . . . . . . . . . . . .
54
8.6.1
ESP-NOW . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
54
8.6.2
Alternatives Considered
. . . . . . . . . . . . . . . . . . . . . . .
54
8.6.3
Data Processing . . . . . . . . . . . . . . . . . . . . . . . . . . . .
54
2
9
Evaluation
56
9.1
Finances . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
56
9.2
Review of Product
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
57
9.3
Future Work . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
58
9.4
Acknowledgments . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
59
3
1
Introduction
1.1
Abstract
Campus exploration can be enhanced by the use of autonomous robots that will elevate
visitor engagement, by following them around, providing useful information, and allowing
user interaction. The aim of this project was to build a two-wheel self-balancing ”Campus
Tour Guide” robot, that manages to maintain stability, while following a designated per-
son through computer vision, or alternatively while being remotely controlled through
an app.
The robot allows users to speak to the app, ask questions, and receive an-
swers through the robot’s speaker and displays additional information about the weather
through the communication between robot and the APP. The project is organized into
six main sections, according to the team’s division of work into modular elements. These
include stability control, manual remote control, battery health display, weather display,
chat mode, and person following using computer vision. The submodules were integrated
on a supplied chassis. Testing after integration confirmed that all general and module-
specific requirements are met, within budget. Thus, the development of a cost-effective
tour guide robot is viable. Future work will explore energy efficiency optimization and
SLAM-based navigation in combination with dynamic path planning.
4
1.2
Background
Balancing robots are canonical examples of nonlinear, underactuated systems commonly
used in control systems research and education. Due to their inherent instability, they
provide an ideal platform for evaluating real-time feedback algorithms, sensor fusion
techniques, and embedded implementations. Their dynamics closely resemble those of an
inverted pendulum, a classic benchmark in control theory.
This project builds on this foundation by integrating complex interactions, including
person-following via computer vision and user interaction through voice chat, into a
balancing robot. It demonstrates how classical control principles can be extended towards
autonomous, intelligent, and interactive robotic systems.
1.3
Product Design
The design of the ”Campus Tour Guide” balance-bot was shaped by principles of mod-
ularity, user interaction, and mechanical robustness, while ensuring all electronic and
software components were integrated in a streamlined fashion.
1.3.1
Chassis and Structural Layout
The robot chassis was provided as a base frame, onto which all electronic modules were
mounted. The design emphasized a low centre of gravity, placing batteries and the motor
driver PCB at the bottom to aid balance. Above this base, the ESP32, Raspberry Pi,
and associated interface electronics were installed in layers, with Velcro and 3D-printed
mounts to allow modular replacement.
1.3.2
Mechanical Modifications
To accommodate the extended application-specific features, structural augmentations
were introduced: a head unit will be 3D printed.
1.3.3
Power Management Layout
The power system was split into two isolated rails: logic (5V) and motor drive (12V),
each regulated and monitored separately. The layout was carefully designed to minimize
noise coupling, with analog circuits for current sensing placed away from high-frequency
PWM motor lines. Decoupling capacitors and a dedicated ground plane were used to
reduce ripple.
1.3.4
Human and Robot Interaction
A strong emphasis was placed on interaction:
• The handle allowed easy repositioning and pick-up of the robot.
• Visual cues, such as a five-bar battery indicator and clear menu UI, enhanced usability.
• Audio output and voice control supported interaction by visually impaired users.
5
1.3.5
Design for Safety and Maintenance
Safety was integrated from the design stage. The chassis was reinforced at high-impact
zones, fuses protected all voltage rails, and the wheelbase was optimized for tilt-resistance.
Additionally:
• Screwed terminals allowed easy part swaps without soldering.
• Over-current and voltage drop-off detection enabled graceful shutdown.
1.3.6
Sustainability Considerations
The product was designed with minimal environmental impact:
• PLA filament was used for all 3D prints, offering biodegradability.
• All modular units are reusable or upgradable.
This product design ensured the robot was not only functionally reliable and safe but
also adaptable for future expansion and improvements.
6
1.4
Project Management
1.4.1
Project Framework Adapted
The Agile Framework was chosen in order to minimize integration risks and ensure rig-
orous project control.
This consisted of one week iterative sprints, which prioritized
high-risk and critical tasks. Frequent reviews and demonstration lead to continuous im-
provement. Cross-functional collaboration between both the hardware and software team
allowed for a quicker integration.
Figure 1: Agile Framework Adapted
1.4.2
Role Allocation
Tasks were allocated in the beginning of the project based on interests, knowledge, and
expertise. Tasks were assigned in a way to ensure that the distribution of the work was
fair. Throughout the development, every team member was part of the integration to
ensure its smoothness.
Table 1.1: Team Responsibilities
Module
Members
Main Controller
Flavio, Lia, Nabiha
LQR Controller
Zecheng
Battery Health Display
Lia, Nabiha
Weather Mode Data Display
Zecheng
Chat AI Mode
Shenghong
Person Following Mode
Flavio
User Interface
Shenghong
Head Unit
Nabiha
Integration
Flavio, Lia, Nabiha, Shenghong, Zecheng
Constant communication was maintained through out the whole project between dif-
ferent modules and each member of the team would help the other members when issues
7
were present, even if they were working on different modules. This ensured high success
and minimal integration issues.
1.4.3
Timeline
A Gantt chart (figure 2) was created to certify efficient allocation of temporal resources.
Schedule successfully remained mostly unchanged throughout the whole project.
Figure 2: Gantt Chart
1.4.4
Weekly Plan
Every Friday online meetings were held to examine individual progression in previous
tasks and assign future deliverables. Rigorous documentation was kept from the start
to track progress, which was kept in a shared folder. Every Tuesday in person meetings
were held to discuss the tasks of the week.
1.4.5
Risk and Contingencies
To ensure proper functionality of the balance-bot project, key risks were identified and
sorted into four categories. These categories define the mitigating actions and fall-backs:
• Hardware & Sensor Risks:
– IMU Failure and Noise: Keep a spare MPU-6050 on hand; verify I2C wiring and
add runtime self-tests.
– Overheating of Stepper Motor and Stalling: Cap maximum step rate in firmware,
stock an extra driver board, monitor stall flags and shut down gracefully.
– Power and Battery Drop: Use a regulated bench supply for early tests and
validate at least 30min runtime on batteries. Implement low-voltage cutoff in
firmware.
• Software and Timing Risks:
– Overloading Control Loop: Enforce a minimum 10 µs gap between stepper driver
pulses; profile and optimise ISR routines; degrade or suspend non-critical tasks
under high CPU load.
• Project and Scheduling Risks:
8
– Part Delivery Delays: Order critical components early and identifying alternative
suppliers in-case of in availability. Begin algorithm development in simulation
while awaiting hardware.
– Scope Creep: Prioritize core requirements before extensible module.
– Communication Gaps: Hold weekly team meetings and maintain concise, up-to-
date documentation in a shared repository.
• Safety and Contingencies:
– Robot Tipping and Damage: Perform initial tests in a contained arena and using
tethered power supply unit for early trials.
– Electrical Shorts: Double-check wiring before each test and using the current-
limited bench supply first.
9
2
Requirements Capture
2.1
Core Requirements, Adaptations, and Verification Bench-
marks
Six core requirements were taken from the GitHub repository [1] shown below. These
capture the basic requirements of the robot and show the system-specific adaptations
used. They also outline how these requirements were modified for this application and
the verification benchmark used.
1. Autonomous behaviour based on camera/sensors
• Adaptation: Person-Following implemented with YOLOv8 nano over a UDP
socket on the Raspberry Pi.
• Verification: Person-following accuracy ≥ 90% over a 5m run.
2. Balance on two wheels with CoG above axle
• Adaptation: Dual PID loops with complementary filter (c = 0.98).
• Verification: Recover from a 10° tilt in ≤ 5 s with ≤ 5% overshoot.
3. Remote-control interface for switching between autonomous and manual
drive
• Adaptation: Flutter mobile app communicating via REST API on the ESP32.
• Verification: Round-trip command latency < 100 ms at 5m range.
4. Display power status (consumption and remaining energy)
• Adaptation: OLED and mobile UI, using a Coulomb-count + lookup-table hybrid
method.
• Verification: State-of-Charge error ≤ 5%, update rate ≥ 1 Hz.
5. UI inputs/displays pertinent to demonstrator
• Adaptation: Four-mode menu (Balance, Weather, Chat, Follow).
• Verification: Mode-switch time ≤ 200 ms; UI refresh rate ≥ 30 Hz.
6. Augment chassis with application-specific head unit
• Adaptation: 3D-printed head unit with 15° camera tilt and 2% manufacturing
tolerance.
• Verification: Field-of-view ±30◦ yaw; handle lift capacity ≥ 2 kg.
These requirements are further expanded into the sections below.
2.2
Stability and Recovery Requirements
These requirements ensure the robot maintains upright posture under various conditions.
1. Stationary Stability
• Test Method: Power on while the robot is held upright.
• Acceptance Criterion: Remain balanced for at least 5 minutes without external
support.
10
2. Straight-Line Stability
• Test Method: Drive the robot forwards and backwards at 0.5m/s.
• Acceptance Criterion: Maintain balance for at least 2 minutes of continuous
motion.
3. Turning Stability
• Test Method: Rotate in place at up to 60◦/s.
• Acceptance Criterion: Remain balanced for the full duration of the battery life.
4. Disturbance Recovery
• Test Method: Deliver a brief lateral push to the robot.
• Acceptance Criterion: Regain upright balance within 5 seconds of the distur-
bance.
2.3
Manual Remote Control Requirement
This implements user-driver motion via controls present on the smart phone app.
1. Move up to 0.5m/s and turn at up to 90◦/s
• Test Method: Log wheel speed and turn rate using the onboard encoder and
gyroscope.
• Acceptance Criterion: Achieve commanded speed within ±5% and turn rate
within ±5◦/s.
2. Control reaction time ≤ 100ms at 5m range
• Test Method: Timestamp command transmission and reception over BLE at 5m.
• Acceptance Criterion: Round-trip command latency remains < 100ms over mul-
tiple trials.
2.4
Extensible Module Architecture
To support extensible feature additions and streamlined testing, software and hardware
were organised into separated modules and each of the extensible module was broken
down such that it could be implemented independently. Each module is summarised
below, with its responsibility, and the extensibility mechanism.
1. Weather Mode Data Display
• Functionality: Fetch local weather via API; present temperature, humidity, etc.
• Performance Benchmark: Display updates within ≤ 500ms after request.
2. Chat Mode Voice Input
• Functionality: Capture speech and speech-to-text processing
• Performance Benchmark: Text processed and outputted within 500ms of speak-
ing.
3. Chat Memory Persistence
• Functionality: Persist summaries of chat prompts and responses in a SQL database.
• Performance Benchmark: Read/write latency ≤ 100ms.
11
4. Battery & Power Status Display
• Functionality: Measure current via differential amplifier; compute state-of-charge
(SoC) and power.
• Performance Benchmark: Update readings every 1s with ≤ 5%SoC error.
5. Person Following
• Functionality: Track the designated person using YOLOv8.
• Performance Benchmark: Maintain a following distance of 1m ± 0.5m.
2.5
Non-Technical Requirements
Beyond functionality, the project imposed organizational and usability constraints. The
details of each of these non-technical requirement and how it influenced our process.
1. Budget Constraint
• Description: Total cost must remain under £60.
• Implementation Impact: Prioritised open-source libraries; 3D-printed head in
PLA to reduce cost.
2. Development Timeline
• Description: Interim demo at week 6; final report at week 12.
• Implementation Impact: Adopted weekly Agile sprints; automated CI checks on
each push.
3. Documentation & Traceability
• Description: Maintain clear documentation for future teams.
• Implementation Impact: Used GitHub for code; refreshed project Wiki after every
sprint.
4. Team Collaboration
• Description: Cross-disciplinary work between hardware and software groups.
• Implementation Impact: Bi-weekly integration meetings; shared GitHub issues
and pull-requests.
5. Accessibility
• Description: UI must be usable by non-technical stakeholders (e.g. visitors).
• Implementation Impact: High-contrast UI theme; simplified menu labels; op-
tional voice prompts.
6. Equity and Inclusion
• Description: Ensure the system is usable by visually impaired users.
• Implementation Impact: Audio feedback; tactile buttons; voice control support.
7. Educational Outreach
• Description: The system should serve as a teaching tool for students and novices.
• Implementation Impact: Included interactive UI overlays and a talk-back feature.
8. Environmental Sustainability
12
• Description: Minimise environmental impact and energy consumption during use.
• Implementation Impact: Low-power sleep modes; uses minimal components.
9. Safety & Compliance
• Description: Electrical and mechanical safety for public demonstrations.
• Implementation Impact: Added emergency-stop button; over-current protection;
CE-compliant wiring.
13
3
Modelling
The dynamics of the robot are described by a mathematical model to facilitate the de-
velopment of an efficient control system. This section derives the equation of motion for
a two-wheeled inverted pendulum with stepper motor actuation.
3.0.1
Stepper Motor Model
The robot uses two stepper motors for wheel actuation. Unlike DC motors, steppers
provide discrete position control through step pulses.
For control system design, we
model the stepper motor dynamics in continuous time with the following assumptions:
• High step resolution (microstepping) allows quasi-continuous motion
• Step frequency is proportional to desired angular velocity
• Motor torque is approximately constant within operating speed range
The simplified stepper motor relationship is:
τm = Ks · fstep
(1)
where Ks is the stepper torque constant (Nm·s) and fstep is the step frequency (Hz),
which relates to the control input.
For the wheel dynamics, we replace the DC motor equations with a direct torque
input model:
HfR = 1
r
�
(Iw + Im) ¨θw − τm
�
(2)
3.0.2
Dynamic model for a two wheeled inverted pendulum
Variable Definitions
Variable
Description
Units
x
Horizontal position of robot
m
ϕ
Pendulum angle from vertical
rad
θw
Wheel angle
rad
Mw
Mass of each wheel
kg
Mp
Mass of pendulum/chassis
kg
Iw
Moment of inertia of wheel
kg·m2
Ip
Moment of inertia of pendulum about center of mass
kg·m2
Im
Motor inertia reflected to wheel
kg·m2
l
Distance from wheel axle to pendulum center of mass
m
r
Wheel radius
m
g
Gravitational acceleration
m/s2
HL, HR
Horizontal forces from left and right wheels
N
τm
Applied motor torque
Nm
Ks
Stepper torque constant
Nm·s
Table 3.1: Two-Wheeled Inverted Pendulum Parameters
14
The robot consists of two main components: wheels and an inverted pendulum chassis.
We analyze each separately then combine them.
Wheel Dynamics
For each wheel actuated by a stepper motor, the dynamics can be described by Equa-
tion (3). Using the kinematic constraint θw = x/r and its differentiation, the dynamics
of the wheel can be rewritten as:
2
�
Mw + Iw + Im
r2
�
¨x = 2τm
r
− (HL + HR)
(3)
Pendulum Dynamics For the inverted pendulum chassis, applying Newton’s laws
in horizontal and rotational directions:
Horizontal force balance for inertial force, centrifugal force and tangential acceleration
forces.
(HL + HR) = Mp¨x − Mpl sin ϕ ˙ϕ2 + Mpl cos ϕ¨ϕ
(4)
Moment balance about center of mass:
Ip ¨ϕ = 2τm
r
− l(HL + HR) sin ϕ
(5)
Linearization For small angles from vertical (ϕ ≈ 0), we use the approximations:
cos ϕ ≈ 1, sin ϕ ≈ ϕ, ˙ϕ2 ≈ 0
This yields the linearized equations:
(Ip + Mpl2)¨ϕ + Mpl¨x = −Mpglϕ + 2τm
r
(6)
�
2Mw + 2Iw + 2Im
r2
+ Mp
�
¨x + Mpl¨ϕ = 2τm
r
(7)
State Space Representation Solving the coupled equations above for ¨x and ¨ϕ, the
final state space model is:


˙x
¨x
˙ϕ
¨ϕ

 =


0
1
0
0
0
0
M2
pgl2
∆
0
0
0
0
1
0
0
−MpglMT
∆
0




x
˙x
ϕ
˙ϕ

 +


0
2Mpl
∆r
0
−2MT
∆r

 τm
(8)
where:
MT = 2Mw + 2Iw + 2Im
r2
+ Mp
(9)
∆ = MT(Ip + Mpl2) − M 2
pl2
(10)
This model assumes no wheel slip, continuous ground contact, and that the stepper
motors can provide the required torque τm as the control input.
15
3.1
Model Predictive Control Approach
Inspired by state-of-the-art control methods [2, 3], we aimed to implement a Model
Predictive Control (MPC) strategy for a two-wheel balancing robot. The cost function
used for discrete-time system dynamics optimization is given by:
J(X, U) =
N−1
�
k=0
�
x⊤
k Qxk + u⊤
k Ruk
�
+ x⊤
NPxN,
where Q, R, and P represent weighting matrices for state regulation.
At each sampling instant, an optimal control sequence is computed based on real-time
state estimation, and only the first control input is applied, with the process repeated
in a receding-horizon fashion. However, due to the computational demands of solving
the optimization problem in real time on embedded hardware, we replaced the full MPC
formulation with its linearized counterpart—the Linear Quadratic Regulator (LQR). The
LQR controller uses a pre-derived optimal state-feedback gain matrix to stabilize the
system.
While LQR present better balancing, experimental results revealed limited robustness
in the outer position loop, especially under external disturbances or model mismatches.
To address this, we drew inspiration from Xu et al. [4], who proposed an reinforcement
learning approach to improve motion planning performance for mobile robot.
A simulation-based training framework was designed using the Twin Delayed Deep
Deterministic Policy Gradient (TD3) algorithm, leveraging the BARN dataset with 300
diverse navigation environments to train this single agent under Gazebo. The sparse
reward function was crafted to penalize tracking deviations and penalize falls, encouraging
robust trajectory-following behaviour. However, due to limited computational resources
on a personal laptop, sim-to-real transfer was not fully executed despite several rounds
of reward tuning and convergence in simulation.
As a result, we ultimately adopted
a robust PID controller for real-world deployment, prioritizing stability and real-time
responsiveness.
16
(a) LQR Controller
(b) Simulation Environment in Gazebo
(c) Training Performance Curve
Figure 3: LQR Controller, Simulated Environment, and Learning Performance.
17
4
Control System
A control system is designed to balance the rover on two wheels, with the centre of
gravity above the rotation axis of the wheels. Additionally, disturbance recovery is also
implemented. Stability is separated into three requirements:
• While standing still - static stability
• While moving forward and backwards
• While rotating
4.1
Design Overview
The controller is designed with dual cascading loop structure to fulfil the balancing re-
quirements mentioned above.
The inner, faster loop consists of two PID controllers,
responsible for stability when standing still and when rotating respectively. It uses the
processed signals from the MPU6050 (accelerometer and gyroscope) as inputs. The outer,
slower loop is a position controller, which uses the average velocity values as input. It is
responsible for the forward and backward movement of the robot.
Figure 4 illustrates this structure.
Figure 4: Control System Structure
4.2
Sensors and Signal Processing
The balancing controller relies on tilt measurement from the MPU6050, the combined
accelerometer and gyroscope sensor.
• 3 Axis Accelerometer: Measures the vector sum of gravitational and inertial
accelerations, and therefore gives the tilt of the robot. However, it cannot distin-
guish between a gravitational force and a force due to acceleration, and hence would
introduce a transient error when the robot moves longitudinally.
• 3 Axis Gyroscope: Measures differential or the rate of change of the tilt an-
gle. However, the output of the sensor has a small error which accumulates when
integrating, leading to drift.
18
Figure 5: MPU6050
These errors are corrected using a complimentary filter (eq 11), which produces a
stable, drift-free estimate of tilt. The complementary filter achieves this by performing a
summation of a low pass filtered accelerometer tilt measurement and a high pass filtered
gyroscope tilt measurement.
θn = (1 − c) θa,n + c
�
θn−1 + ˙θg,n ∆t
�
(11)
Here, θa,n is the accelerometer-derived tilt at timestep n, ˙θg,n = dθg
dt is the gyroscope’s
angular rate, ∆t is the sampling interval, and c ∈ [0, 1] is the filter constant.
The filter constant is chosen to be c = 0.98, to emphasize the gyroscope dynamics,
allowing accurate measurement for rapid tilt variations and limiting accumulated error
bias.
4.3
Inner Loop PID controller
The inner loop repeats every 20 ms and implements two separate proportional integral
derivative (PID) controllers. The controller’s purpose is to minimize the error e(t) which
is calculated as the difference between the robot’s current angle and the set-point while
avoiding a large overshoot.
The PID controller sums the error multiplied by a constant Kp, the derivative of the
error multiplied by a constant Kd, and the integral of the error multiplied by a constant
Ki (Equation 12). This produces the control output u(t), which is the velocity command
needed to drive the system output toward the desired set-point.
u(t) = Kp e(t) + Kd
de(t)
dt
+ Ki
� t
0 e(τ) dτ
(12)
In the PID controller:
• linear term indicates the primary corrective torque
• derivative term indicates the damping movement and is preventive of oscillations
• integral term eliminates the static error, which is the steady-state offset
19
Inner-loop error calculation
err_inner = (ref + tiltSP) - theta;
integral
+= err_inner * dt;
deriv
= -gyro_y;
4.3.1
Static PID
The parameter values chosen after tuning, shown in Section 4.2, are as follows:
Kp = 1000,
Kd = 200,
Ki = 1.
These values imply a control strategy where the proportional term dominates, ensuring
rapid correction of any deviation in the error.
The derivative term is high, but has
less contribution, as it provides damping to reduce overshoot and makes the controller
resistant to sudden changes in the angular velocity. The integral gain is relatively much
smaller and gradually eliminates steady-state error. It is limited to a small value to avoid
high overshoots, but still have enough static error correction to return to the desired
position.
A complementary filter, as shown in Section 4.2 combines the accelerometer z and x
axis readings and the pitch of the gyroscope to find the robot’s angular distance from the
reference (θ).
4.3.2
Rotation PID
The rotation PID controller is based on the same logic as the standard inner loop but uses
the gyroscope roll output to track its position. It implements this by using the previous
rotation angle and adding how much it has changed since the last measurement.
Rotation angle calculation
rotpos = prev_rotpos + spinRate * dt;
It then uses the rotation error, the rotation errors derivative and its integral for the
controller.
Periodic control loop within the INNER INTERVAL
spinErr
= h_webDesired - rotpos;
spinDeriv = (spinErr - prev_spinErr) / dt;
spinIntegral += spinErr * dt;
As in a standard PID, each of these values are multiplied by their respective gain
values and summed together.
Periodic control loop within the INNER INTERVAL
float Prot = rp * spinErr;
float Drot = rd * spinDeriv;
float Irot = ri * spinIntegral;
float rotvel = Prot + Drot + Irot;
20
This controller has the following gain values:
Kp = 2,
Kd = 0.5,
Ki = 1.
The rotation controller can be implemented with much lower gain values because the
rotation speed is considerably lower than the speed required to keep the controller stable.
This rotvel output would then be added to the inner stability loops target speed of one
wheel and subtracted from the target speed of the other wheel, implementing rotation.
Periodic control loop within the INNER INTERVAL
step1.setTargetSpeedRad( uout
- rotvel );
step2.setTargetSpeedRad( uout
+ rotvel );
4.4
Outer Loop controller
To ensure that the robot can hold and attain a desired position, a slower outer is loop
implemented, which repeats every 100ms. The velocity of the rover is increased and
decreased by varying a constant which is added to the reference tilt, which would lead
the inner loop controller to make the rover speed up or slow down in order to prevent
it from falling over. In order to avoid an exaggerated tilt, constraints on the maximum
angle are included, by which the reference angle is varied.
The way this controller works is that it is separated into 3 main parts, one for when the
robot is on or is very close to the desired position, one for when the robot was within
a small bound of the desired position and one when it was very far away. For this we
used 2 thresholds separating these stages into three stages, with the distances of 10 and 5.
The way the furthest distance mode works is that the loop uses an average velocity
calculated over the past 5 iterations of the inner loop to see how fast it is moving. This
value is then reset to 0 after every time it is used in the outer loop.
Speed averaging calculation
speedcount++;
SumSpeed += 0.5f * (sp1_meas + sp2_meas);
avgSpeed = SumSpeed / speedcount;
It then uses this to edit the speed, and then the system works to try and achieve this
specific speed. The direction of the rover would be determined by the sign of the position
error (desired position - actual position), making it move towards the left side if it was
too far right and vice versa.
Periodic control loop at OUTER INTERVAL
if(abs(posErr) > POSERRLIMIT){
if(posErr < 0){
21
Kp = (-0.05)/(2);
}else{
Kp = 0.05;
}
tiltSP = -constrain((Kp/abs(avgSpeed)),
ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT);
}
Once the position error is small enough, this passes the outer threshold, after which it
uses a combination of the velocity and the magnitude of the position error to slow down
faster as it gets closer to its desired position. This is done to avoid an overshoot.
Periodic control loop between Outer INTERVAL and INNER INTERVAL
else if (absErr > POSERRSLOWLIMIT) {
if(posErr < 0){
Kp = (-0.025)/(2);
}
else{
Kp = 0.025;
}
tiltSP = -constrain((Kp/abs(avgSpeed) *
((abs(posErr)-5)/10)), -ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT);
}
After the robot gets even closer to the desired position, it would pass the inner thresh-
old, after which it would no longer take into account the outer loop controller. This is
implemented by setting the outer loop value added to the reference equal to 0, and would
simply remain stable within the desired region.
Periodic control loop within the INNER INTERVAL
if(absErr < POSERRSLOWLIMIT){
tiltSP = 0;
}
One thing to note about this approach is that it causes the velocity of the rover to
oscillate around the desired value as demonstrated in figure 6.
22
Figure 6: Plot of the velocity of the rover against time, current method
This method works successfully to achieve stability while moving, but the rover slows
down and then accelerates repeatedly.
A future development to avoid this would be to make it such that the rover only
initially receives a tilt. This would lead to it accelerating in the desired direction, and
soon after, it will go back to a tilt close to the usual reference (slightly inclined to take
into account friction and air resistance). Since it has now achieved its desired velocity, it
can then keep moving at this constant velocity until it is near the desired position. After
this, it receives the same magnitude of tilt as before but in the opposite direction such
that decelerates and goes back to being stationary.
This method is graphically described in figure 7 which shows the tilt and figure 8
which models the velocity.
23
Figure 7: Plot of the tilt of the rover against time, method for future implementation
Figure 8: Plot of the velocity of the rover against time, method for future implementation
24
4.5
Parameter Tuning
Multiple methods were employed to tune the parameter, to find the most suitable values
for the PID Controller (KP, KI, KD) that produce most stable behaviour. This was done
in multiple stages:
• Simulation
• Loop Shaping
• Trial and Error
4.5.1
Simulation
Multi-joint dynamics with contact (MuJoCo) Python-Tkinter simulation was developed
to accelerate the tuning process. The robot was simulated as a body on two wheels, with
the centre of gravity above the rotation axis of the wheels, mimicking the robot.
Rapid trial and error was performed to tune the PID gains and balancing stability
was achieved in an ideal, frictionless environment in the simulation.
Although, the physical robot did not balance when tested with these PID gains, it
provided a good starting point for loop shaping and significantly reduces the time required
for hardware parameter tuning.
The robot did not balance in real word due to air resistance, surface friction, noise in
the sensor, overheating of the components, vibrations in the chassis and the motors dead
zone.
(a) Simulation Parameters Tuning
(b) Simulation of the robot
Figure 9: MuJoCo Simulation Trial and Error
4.5.2
Trial-and-Error Tuning
Balancing was achieved by tuning the parameters of a default PID controller.
25
On the real robot, the initial gain values were set to those found using simulation and
then were iteratively adjusted. These adjustments were made based on the behaviour of
the robot observed.
If there was an overshoot. which can be observed when the robot runs and falls over,
then the proportional gain was increased to provide a larger restoring force.
If the robot oscillated before falling over, then either the proportional gain was de-
creased or differential gain was increased to introduce damping and remove overshoot.
The integral gain was kept relatively low while tuning. This is because a reference
was added to the inner loop which indicated the the right set-point for static conditions.
This reference accounts for the offset of the centre of mass of the robot from the central
axis used to measure tilt and any misalignment between the tilt sensor and axis. This
offset was found by turning the motors off and finding the tilt angle measured when the
robot naturally balances as it is gently held upright.
A general guideline used for trial and error is displayed in table:
Gain
Rise Time
Overshoot
Settling Time
Steady-State Error
Kp
Decrease
Increase
No effect
Decrease
Ki
Decrease
Increase
Increase
Eliminate
Kd
No effect
Decrease
Decrease
No effect
Table 4.1: Effects of PID Gains for Tuning
4.5.3
Loop-Shaping Control Design (Considered)
Loop-shaping is a frequency-domain method that designs a controller C(s) to shape an
open-loop transfer L(s) = C(s)G(s) which meets the robustness and performance targets
of the robot which include the bandwidth, phase-margin, disturbance-rejection and noise
attenuation.
Although this method was not used to structure the controller for reasons mentioned
later, a typical workflow followed would be:
1. Model identification: Obtaining a low-order model of the system from step responses
or frequency sweeps.
G(s) ≈
K
(T1s + 1)(T2s + 1)
2. Specification definition: Choosing a desired crossover frequency ωc, phase-margin
ϕm, and sensitivity bounds (e.g. |S(jω)| < 0.1 for ω < 1 rad/s). These values would
be chosen based on the time domain requirements of our system including settling
time, max overshoot, and acceptable steady-state error.
3. Compensator structure: Selecting a cascaded lead/lag and roll-off sections, for ex-
ample
C(s) = Kc
s + z
s + p
1
τfs + 1 ,
placing z, p around ωc and τf to tame high-frequency noise.
4. Loop-shape verification: Obtain the Bode plot of L(s), confirm |L(jωc)| = 0 dB,
ϕm ≥ 45◦, and the desired disturbance-rejection and noise-attenuation is achieved.
26
5. Discretization. Converting C(s) to C(z) via the Tustin transform for implementa-
tion at an appropriate ∆t ms.
Although loop-shaping give precise margins and formally guarantees robustness,this
method was not used due to the following limitations:
• Uncertainty in the model since the real robot exhibited modelled friction, motor
dead-zones, and chassis flex that our simple G(s) fit could not capture reliably.
• time constraints of the project, since accurate system identification and iterative
frequency-domain tuning would be more time-consuming than the rapid trial-and-
error approach adapted.
• This method has more computational complexity since a loop-shaping compensator
built from multiple filter stages, including the cascade lead/lag and the roll-off,
requires more multiply–add operations each cycle, significantly increasing CPU load
on the control loop and adding effort in coefficient scaling and validation.
As a result, the trial-and-error method was prioritized, which achieved practical bal-
ance performance more efficiently in the available time-frame.
27
5
Manual Remote Control
Rather than discrete arrow-buttons, the app can present a single 2D slider (joystick)
control as shown in figure 25. When in manual remote control mode the user drags the
central thumb within a circular boundary to select direction in which the rover should
move/ rotate; releasing it snaps back to the centre, freezing motion.
Operation:
• Push Up:
– Maps vertical displacement y > 0 to a forward tilt offset:
g webDesired = pos − Kv y
where Kv scales the drag distance to a tilt bias. This causes the robot to roll
forward with speed proportional to y.
• Push Down:
– Maps y < 0 to a backward tilt offset:
g webDesired = pos − Kv y
(note y < 0 yields a positive offset), driving the robot in reverse.
• Push Right:
– Maps horizontal displacement x > 0 to a heading bias:
h webDesired = spinComp + Kh x
where Kh scales x to a yaw offset. This produces an in-place spin to the right at
rate ∝ x.
• Push Left:
– Maps x < 0 to a leftward heading bias:
h webDesired = spinComp + Kh x
(with x < 0), causing an in-place spin to the left.
• Neutral (Released):
– Returns to center (x = y = 0), so g webDesired = pos, h webDesired =
spinComp. Both position and heading set-points “freeze” at their current val-
ues.
Implementation details:
• Continuous updates: As the user drags the thumb, the app sends a REST call every
100ms with the current (x, y) normalized to [−1, 1].
• Server mapping: On the ESP32, handleCmd() parses a JSON payload {"x":...,"y":...}
and computes the two set-points according to the equations above.
• Smooth control: The inner balance-PID and spin-PID loops blend these biases into
wheel-speed commands, yielding smooth acceleration, deceleration, and turning pro-
portional to joystick deflection.
28
6
Head Unit
The head unit was designed such that it provided good utility, ease of use and mounted
the camera at an angle suited for best performance. The design process consisted of
iterative sprints, which allowed testing and continuous improvement.
6.1
Utility
The head unit was designed such that it has a handle, allowing the user to easily pick
it up. It also consists of a camera holder, to mount the camera and protect it, while
providing an easy connection to the raspberry Pi.
6.2
Design Process
The holder base was designed using references from the fusion model of the logic module
of the robot provided on GitHub [1]. This enabled an accurate size of the head unit. The
camera used is the Raspberry Pi V2.1 camera module [5]. The data sheet and a CAD
module of the camera [6], shown in figure 10 , were used to accurately design a camera
holder, such that camera fits perfectly inside the holder and the sides of the holder area
have a suitable width to provide sufficient protection in-case it falls.
Figure 10: CAD model of camera unit
The sides of the inside of the holder were designed such that it provided a snug fit
to the holder, shown in figure 11 . The handle is designed in order to hold the weight
of the rover, and therefore the infill density is chosen to be high, set at 50 percent with
honeycomb structure, which allows for isotropic strength, which means it withstands
forces from every angle.
29
(a) Front View
(b) Side View
Figure 11: CAD model of camera fitted to head unit
6.2.1
First Design
In the first design, the holder was designed on a right angle to the base. However, the
handle designed was too low, and did not have enough room, hence making it harder to
pick up.
6.2.2
Second Design
In the second design, tolerances of 2 percent were added to the camera holder to allow
for friction and small inaccuracies in printing. Moreover, the design of the handle was
changed so that it was higher, allowing the user to easily lift it off the ground. However,
it was constrained such that it does not hit the floor if the robot topples over, to prevent
breaking.
Figure 12: CAD model of second design
30
(a) Printed Head Unit
(b) Head unit with camera
Figure 13: Second Design Models and Printed Version
6.2.3
Third Design
In the third design, handle design was kept the same since it was easily able to lift up the
rover when tested. However, the camera holder design was altered. A 15 degrees tilt was
added so that the back of the users head is clearly visible to the camera. it would also
enable the robot to closely follow behind the person, hence allowing clear communication
via the chatbot module.
Moreover, using the data sheet and the CAD model, holes
were added to the camera holder to allow a secure connection and preventing the camera
from falling by screwing it securely to the head, during a fall or even from very strong
vibrations.
(a) Front view
(b) Side view
Figure 14: CAD model of third Design
31
Since this design met all the utility requirements, it was chosen as the final version
implemented. Figure shows the final version of the head mounted on top of the robot,
both in CAD and on the actual hardware.
(a) CAD Design
(b) Physical Robot
Figure 15: Head unit fitted onto the robot
32
7
Battery Analysis
One of the base requirements for the project is to display useful information about the
power status of the robot. Hence, battery health and the instantaneous power consumed
by the robot is displayed on an OLED screen mounted on the rover and on the app. The
battery health is found using the state of charge remaining. These calculations are done
using the voltage measurements and current sensing using a differential amplifier.
7.1
Coulomb Counting
The first method used to calculate the state of charge is Coulomb Counting, which uses
current flow to measure battery health. In this approach, the charge consumed by the
robot is found by integrating the current over the operation time. In this case, the instan-
taneous current is found through current sensing differential amplifiers as will be explained
below. The accumulation of the charge consumed is subtracted from the nominal charge
of the battery. The charge remaining in the battery is expressed as a percentage, which
gives the real-time SoC (State of Charge.
Qconsumed(t) =
� t
t0
I(τ) dτ.
(13)
This operation can be performed in software either by the Riemann sum or Trapezoidal
sum. Trapezoidal sum is chosen, as it yields more accurate result for changing current,
since it reduces the truncation error that is present in Riemann sum. Figure 16 graphically
demonstrates the difference in accuracy between the two integration methods.
Figure 16: Accuracy Difference between Trapezoidal Sum and Riemann Sum respectively
Total charge consumed is found by iteratively adding the instantaneous charge con-
sumed to the same variable.
Qtotal(t) = Q(t) + Qprevious(t)
(14)
After the battery is fully charged we initialize the value of Qprevious(0) = 0
Then, the battery health (SoC) is calculated knowing the battery’s nominal capacity
Cnom = 2000mAh,
SoC(t) = Cnom − Qtotal(t)
Cnom
× 100%
(15)
33
7.2
Discharge Curve Method
The second method to calculate battery health percentage employs the raw battery volt-
age. A look-up table is generated using the discharge curve from the data sheet of the
battery. [7].
This discharge curve figure 17a in the data sheet is for one 1.2V battery. In our setup,
the battery pack consists of six of these batteries. Furhter, two battery packs are used in
the robot, connected together in series. Hence, the voltage in the original discharge curve
is multiplied with 12 while the capacity (QAh) remains the same. Hence, the discharge
curve in figure 17b is obtained, which is accurate to the robot.
The design decision was made to use the discharge curve in the data sheet directly, as
it would be a more accurate representation of the average discharge of the batteries. If
the discharge curves were obtained manually, by discharging the batteries completely, it
would be less accurate since each battery is different from the other. The discharge curves
are at 15A current. Although the current we use is lower, this method is acceptable, since
the lower current simply reduces the IR drop, which only translates the curve upwards,
without changing its shape.
The state of charge is obtained for each value of V, using the time as follows:
Qconsumed = 15 A × time (min)
60
(16)
SoC = Qnominal − Qused
Qnominal
× 100
(17)
A state of charge percentage is found for multiple points which corresponds to a certain
battery voltage, and the results are tabulated to form a look-up table for battery health
calculation. This look-up table has been plotted in figure 18
Figure 18: SoC lookup curve for 12-cell battery pack
Measuring the battery voltage, as shown in section 7.5.3, we compare this voltage to
the look-up curve, and are able to find the SoC charge remaining.
34
(a) Original Discharge Curve from datasheet
(b) Discharge Curve for battery pack
Figure 17: Comparison of battery discharge curves
7.3
Evaluation of Methods
The decision of which method to use was made after considering the limitations of both.
The major limitation for Coulomb Counting was that the position of the zero is
unknown when we begin counting.
It is also limited by the accuracy of the current
sensing circuit and any offset needs to be carefully accounted for, since it could greatly
affect the error if integrated into the term.
On the other hand, the limitation for the discharge curve is that a very flat region
is present in the central range of operational voltages, which is where most of the robot
operation will be conducted. This loses much required accuracy. However, at both the
extremes of the operation region, the curve is much steeper, hence providing a much more
accurate measurement of the SoC.
Thus the most accurate method is to use the discharge curve to find the initial zero
35
for the coulomb count when the robot is powered on, and then proceeding with the
coulomb counting from there.
When the robot will be powered on, it would mostly
be freshly charged, and hence would be on the extremes of the operation range, thus
making the discharge curve an accurate measurement. This also helps to eliminate further
inaccuracies because when the batteries are recharged, they would not be at a 100%. This
would enable it to find an accurate percentage for the SoC at the start.
7.4
Power Consumption
The instantaneous power consumed is also displayed. This is obtained by by multiplying
the instantaneous current with the voltages.
Pcons = I(t)inst.V
(18)
This is found by summing the power consumed by the logic module and motor module,
since these currents are measured separately.
For the power of the motor module, spikes occur in the current measured when the
motors are running. Hence, the power is measured and added over 5 seconds and then
averaged to display a readable and user friendly value, while still taking into account the
spikes.
7.5
Implementation
7.5.1
Current Sensing Circuit
To enable the above mentioned analysis, a current sensing configuration is needed. The
current is measured using a differential amplifier, with one input as the voltage source and
the other across a small shunt resistor present on the PCB. The relevant shunt resistors,
across which the voltage drop is measured, are R1 and R3 , as shown below,
Figure 19: Power PCB Schematic
36
Since the voltage difference is a a few mili-volts, significant amplification is needed.
This amplified voltage is then sent to the analogue to digital converter (ADC) in the
breakout board and processed in software. The ADC in the breakout board is used, since
it is more accurate than the one in the ESP32.
The default differential amplifier, [8] as shown in Figure 20 is used with some minor
modifications. The offset bias is adjusted to be 1.25V, via a potential divider connected
to R4 with a unity buffer, which enables load isolation. However, the true offset at each
amplifier circuit varies slightly away from 1.25V. This reflects the input offset voltage
of the chosen opamp, which is shown in the datasheet to typically be 0.5mV and at
maximum 5mV [9] . The present offset is indeed in this range. Hence, this offset is found
using by tying both inputs together and is then used in firmware for accurate current
measurement.
Figure 20: Standard Differential Amplifier Schematic
The LTspice schematic is shown below in figure 21,
37
Figure 21: LTspice schematic of two differential amplifiers responsible for current sensing
The hardware implementation of the circuit is also shown below in figure 22,
Figure 22: Hardware Implementation of Current Sensing Circuit
7.5.2
Sensing Circuit & Calibration
Since the battery rails are ≤ 5V , they can be directly used as the inputs to the differential
amplifier. However, the motor rails are ≤ 15V , and thus they need to be scaled down.
This is achieved via two 1:3 potential dividers, connected to inputs via unity buffers for
load isolation, and to ensure that the gain remains as calculated. Capacitors are added
in parallel to the second resistor in each potential divider to limit high frequency noise,
since all signals are DC valued. The value for a suitable resistor is calculated such that
it remains below the time constant. Additionally, an inductor is also placed before the
38
first resistor to form a second order low pass filter, resulting in steeper roll off and greatly
reduced the noise. The roll off is around 1KHz, resulting in much clearer ADC readings.
A decoupling capacitor is also used between the power rails on the breadboard that
are used to power the opamps, in order to limit supply noise.
The opamps chosen are the T272I [9], since they fullfill all design requirements. They
supported rail-to-rail inputs, have a sufficiently high gain bandwidth product (3MHz)
and a fast enough slew rate (2.7 V/µs ) for the application.
When the resistors R1 = R2 and R3 = R4, the gain can be calculated by:
Gain = R3
R1
× (V+ − V−)
The gain selected is:
Gainlogic = 510; Gainmotor = 100
(19)
These values are chosen such that the voltage can be read with enough precision according
to the accuracy of the ADC present in the breakout board.
The accuracy is 20 mV
calculated using equations below:
Bits = 12; Vref = 4.096V
(20)
∆(step) = LSB = 4.096
212
= 10mV ; Accuracy = ±1LSB = 20mV
(21)
Moreover, the bias was added so that the final readings lie in the middle region of the
voltage conversion range.
Once the voltage difference is measured it is converted back into current with the
following equation:
I = ∆V − offset
gain
÷ Rshunt
(22)
For the motor side, ∆V is multiplied by 4 to take into account the scaling. The final
values used are in the table below:
7.5.3
Voltage Measurement
The voltages are measured with a 1:3 potential divider circuit to get the voltage ≤ 4V ,
so they are safe to send to the ESP board. The value is fed to the ADC pin via a 10 kΩ
resistor, to limit the current and ensure the ESP board remains safe.
7.5.4
Display
The OLED displays the battery health, as a percentage and a configuration with a battery
consisting of 5 bars, which indicate how full it is. The instantaneous power consumed
is also demonstrated, as well as the voltage from the logic and for the motor side of the
difference amplifier. When the switches are turned off, the true offsets are shown as the
voltage. In that case no current passes through, so the power is equal to zero. When the
switches turn on, there is a jump in power and thus the voltages increase.
39
(a) Display when both switches are turned off
(b) Display when both switches are turned on
Figure 23: Display of the battery health, the instantaneous power consumed, and the
differential voltage
The app also displays real-time battery usage information on the top-right corner
through sending periodic HTTP GET requests to the interface ESP32.
Figure 24: App UI with power status of the robot
40
8
User Interface
8.1
Cross Platform Mobile App
The robot will be interacted through a Flutter-based mobile app that is available on both
iOS and Android. The app has four modes to choose from:
1. Weather UI
2. Person Following
3. AI Chat
4. Remote Control
(a) Weather UI
(b) Remote Control
(c) AI Chat
Figure 25: Different Modes: (a) Weather UI, (b) Remote Control, (c) AI Chat
As the robot requies internet access in modes such as AI Chat and Weather UI, the
mobile device’s hotspot network will be used to connect to the Raspberry Pi and ESP32.
41
8.1.1
Mode Switching
This feature is implemented with a REST API hosted on the ESP32 as its stateless and
unidirectional. When a mode is chosen, its respective function will be run on the ESP32.
int currentMode = 2; // Default weather UI
AsyncWebServer server(80);
server.on("/set_mode", HTTP_POST, [](AsyncWebServerRequest *request) {
if (request->hasParam("mode", true)) {
String modeStr = request->getParam("mode", true)->value();
...
} else {
request->send(400, "text/plain", "Missing 'mode' parameter");
}
});
This AsyncWebServer implementation ensures that the ESP32 switches mode imme-
diately upon request from the app.
8.1.2
On-device Capabilities
For accessibility, the robot will use the mobile device’s microphone for AI Chat mode.
The mobile device’s speech-to-text functionality will also be used to send text prompts to
OpenAI Chat Completion API. When using Weather UI mode indoors, on-device GPS
will be used for determining latitudinal and longitudinal values.
8.2
AWS EC2 Server
The server uses Django REST API for a consistent and standardised interface that is
easy to use. Functionalities include creating, reading, updating and deleting conversa-
tion memory and location data. The framework also offers built-in authentication and
permissions, ensuring communication and data security.
Figure 26: Example HTTP GET request from the server
8.2.1
JWT Authentication
Each robot is associated with credential information tied to its respective user account.
Upon successful login, the authentication server issues a JSON Web Token (JWT). This
token is then included in each HTTP request to access protected resources, such as the
memory of past conversations.
42
8.2.2
Database
SQL databases are preferred over NoSQL as there are relational data stored in the
database. The system also has write-heavy workloads relating to storing and deleting
past conversation memories in AI Chat mode. Hence, PostgreSQL is chosen over MySQL.
Figure 27: Entity Relationship Diagram
8.2.3
Admin Website
An admin page is necessary to track the GPS location of the robot and view data stored
in the database on the server. Hence, a ReactJS frontend is built to visualise all robot’s
GPS location movement. Administrators will also be able to view data such as chat
memories and maintenance records.
Figure 28: ReactJS Admin Page and Map
43
8.3
Weather UI Mode
The campus tour guide is given the ability to provide users information about the weather.
Local weather data, such as temperature, humidity, luminance and pressure is periodically
read by the ESP32 from the weather sensor GY-39. The ESP32 also acquires the robot’s
GPS coordinates through the GT-U12 GPS module. These information are sent to the
mobile app on request through the REST API hosted on the ESP32. The mobile app also
requests for the mobile device’s location before sending a HTTP POST request to Google
Weather API. The app then displays both real-time, hyperlocal weather data given the
mobile device’s location and weather data from the robot’s sensors.
Figure 29: Communication System for Weather UI Mode
8.3.1
Weather Sensor
The low-cost weather sensor module GY-39 is used to identify weather information, and
is mounted on the rover. Every few seconds the ESP32 requests the sensor via UART for
temperature, humidity, pressure and altitude data through memory address 0xA5 0x52
0xF7 and light intensity in 0xA5 0x51 0xF6. The reply of the sensor is a fixed-length
packet with the prefix 0x5A 0x5A.
This response is then converted into standard units. It is stored with a timestamp and
error-check flag for validation.
8.3.2
GPS Coordinate Processing
The GPS module’s output is read by the ESP32 via UART, where a standard NMEA
sentences like $GPRMC or $GNRMC is received. Comma-separated fields is parsed in order
to extract the UTC time, latitude, and longitude in degrees and minutes format, and the
direction indicator (North/South, East/West). These get converted into signed decimal
degrees representing the robot’s current latitudinal and longitudinal coordinates.
44
8.3.3
Google Weather API
Upon switching to Weather UI mode or refreshing the app, a HTTP POST request with
the mobile device’s GPS coordinates is sent to Google Weather API. This is followed by
a JSON response containing information regarding real-time temperature, humidity and
perceived temperature.
8.3.4
GPS Signal Contingency
In the case of weak GPS signal on the robot, continuous operation can be achieved by
the use of coordinates of a predetermined default location. Timestamps comparisons of
the last successful GPS update can trigger location failover, once data is outdated. This
ensures that the location based functionality is uninterrupted.
8.3.5
Display
In weather mode, current device represents outdoors weather data and robot represents
actual surrounding temperature and humidity (useful when indoors). The sensor’s data
are retrieved using a HTTP GET request to the ESP32 REST API:
server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request) {
String response = weather(); // to access data from sensor
request->send(200, "application/json", json);
});
Figure 30: Weather Card
8.4
AI Chat Mode
As the campus’ tour guide, the robot is capable of introducing various school buildings,
their history, related departments and offered courses. Hence, OpenAI Chat Completion
API will be used to produce audio responses to user’s questions. The app also stores
and retrieves current conversation’s memory through the AWS EC2 server. The audio
response can be outputted through the robot or mobile device speaker.
8.4.1
Prompting
The app utilises the mobile device’s in-built microphone and speech-to-text capabilities
to convert audio to text. The app then sends a HTTP GET request to the server for the
current conversation’s memory, which will be added to the prompt. Retrieval-Augmented
Generation (RAG) context is implemented in the system role of the prompt.
45
Figure 31: Communication system of AI Chat Mode
messages = [
{"role": "system", "content": rag_context}
{"role": "user", "content": memory_block},
{"role": "user", "content": current_user_message}
]
8.4.2
Response
The user can opt to play the response audio through the robot or mobile device speaker.
The app can either use the in-built speaker of the phone or send the MP3 file through
the REST API hosted on the ESP32. Upon receiving the MP3 file, the ESP32 runs:
audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT); // setup MAX98357 I2S module
server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
request->send(200, "text/plain", "File received");
}, [](AsyncWebServerRequest *request, String filename, size_t index,
uint8_t *data, size_t len, bool final) {
static File uploadFile = SD_MMC.open(filename, FILE_WRITE);
uploadFile.write(data, len); // Store the MP3 file in the SD Card
audio.connect(filename); // Read from SD Card
chat(); // Play new received audio
});
8.4.3
Chat Memory
Summaries of both the prompt and response are used for full context reconstruction.
Before sending the prompt to OpenAI API, a HTTP GET request is sent to get the
current conversation’s memory block. The prompt will have the following example format:
Previously in this chat:
1. User asked about dormitory options → You described available dorms.
2. User asked for directions to the library → You gave walking directions
from the main gate.
3. User asked about dining hall hours → You listed breakfast, lunch, and
dinner times.
Upon receiving a response from OpenAI API, a HTTP POST request will be made
to the AWS EC2 server to summarise and store the text prompt and response. The user
46
is also given the option to start a new conversation, which deletes all memories stored in
the database.
8.4.4
Retrieval-Augmented Generation (RAG)
The app requests context for RAG from the server by sending a HTTP GET request. The
context is stored on the server to allow administrators to edit when necessary. Currently,
the sever stores departmental information such as courses and modules offered.
Example System Prompt
You are a helpful, friendly, and knowledgeable AI tour guide for
Imperial College London.
You will receive retrieved context to help answer each user's question.
- Use the retrieved context to provide accurate and updated information.
- If the context does not contain enough information, politely say you
don't know, rather than guessing.
- When giving directions, be as specific as possible, using landmarks
or building names.
- Keep responses brief and student-friendly, as if you are guiding a
new visitor on campus.
- Answer in natural and clear language, avoiding overly technical terms
unless asked.
Context is provided below: // response containing context from server
47
8.5
Person Following Mode
In this project, we implement a distributed person-following system for a balance-bot.
The high-resolution Pi Camera v2.1 provides visual input, which the Raspberry Pi client
downsizes into lightweight thumbnails.
These are streamed over ZeroMQ to a GPU-
equipped laptop running YOLOv8 for person detection. The laptop publishes bounding-
box data back to the Pi, which rescales the chosen detection and forwards only the
horizontal offset and area to an ESP32 via UDP. The ESP32 then integrates these val-
ues into its inner/outer PID loops to steer and balance the robot toward the target
person.Figure 32 shows a person being detected wihh 92% accuracy.
Figure 32: App UI for Person Following Mode
48
8.5.1
YOLO Algorithm
In order to achieve both good time efficiency and high accuracy we decided to use the
pre-trained YOLOv8 “nano” model for person detection. Inference is invoked on the
laptop as follows:
Inference Call
res = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]
boxes = res.boxes.xyxy.cpu().int().tolist()
Here, ‘classes=[0]‘ restricts detection to people, and ‘conf=0.35‘ filters low-confidence
boxes. The server then packages results:
Publish JSON
pub.send_json({"t": ts, "boxes": payload})
where payload is a list of [x1,y1,x2,y2,confidence] for each detection. By streaming
only thumbnails and a lightweight JSON, we keep round-trip inference latency under 50
ms.
Figure 33: Latency Values
The person tracking is implemented such that people of different heights can be de-
tected accurately as shown in figure 34. The accuracy with which they are detected can
be seen by the number mentioned above the green square in the pictures. It was ensured
that wheel chair users are also detected accurately, by having one person sit on a low
chair while testing.
49
(a) Person detection with two people
(b) Person detection with three people with
one sitting down
Figure 34: Person Detection Testing
50
8.5.2
Server Setup (CUDA)
Before running inference, we detect and configure CUDA, then load the YOLOv8-nano
model onto the GPU in FP16 for maximum throughput:
Server Setup (CUDA)
import torch
from ultralytics import YOLO
import pathlib
print("CUDA available:", torch.cuda.is_available())
device = "cuda" if torch.cuda.is_available() else "cpu"
MODEL_PATH = pathlib.Path("models/yolov8n.pt")
# Load and move to GPU (FP16) if available
model = YOLO(str(MODEL_PATH))
model.model = model.model.to(device).half()
8.5.3
Streaming Architecture
Pi → Laptop (Thumbnails)
On the Pi, we capture a 320×240 JPEG thumbnail
every 1/15 s and PUSH it:
Pi-Client Thumbnail Send
thumb = cv2.resize(frame_rgb, (320,240))
packet = struct.pack("<d", now) + \
J.encode(thumb, quality=70,
pixel_format=TJPF_RGB)
push.send(packet, flags=zmq.NOBLOCK)
push = ctx.socket(zmq.PUSH) connects to the server’s PULL on port 5555.
Laptop → Pi (Detections)
After inference, the server PUBlishes JSON on port 5556:
Server PUBlish
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
# ...
pub.send_json({...})
The Pi runs a background thread to SUBscribe:
Pi Client SUBscribe
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://<laptop-ip>:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")
8.5.4
Designated Person Selection
When multiple detections occur, we had 2 methods to decide who to follow, the main one
is to send a stream of people to the UI, where the user can decide which person the rover
51
should follow by selecting thier ID, however in the case that this fails the fallback was to
choose the largest box (closest person) given that when the robot is following someone
around campus they should be the closest ones to the rover, especially on its line of sight
(direction of the camera).
The way the person tracking would work is the following way:
the laptop, after running YOLOv8 on each thumbnail, we feed the raw bounding boxes
into a SORT tracker instance. SORT attaches a track id to each box and maintains it
across frames:
YOLO + SORT Tracking
from ultralytics import YOLO
from sort import Sort
model
= YOLO("models/yolov8n.pt")
tracker = Sort(max_age=5)
res
= model(img, imgsz=320, conf=0.35, classes=[0])[0]
boxes
= res.boxes.xyxy.cpu().numpy()
confs
= res.boxes.conf.cpu().numpy()
dets
= np.hstack((boxes, confs[:,None]))
tracks
= tracker.update(dets)
SORT uses a Kalman filter + IOU matching to keep IDs stable when people move or
briefly occlude.
After this the user would select which person it wants the robot to follow leading to
the following check:
User-Selected Person
for box, tid in zip(boxes, ids):
if tid == chosen_id:
x1,y1,x2,y2 = box
break
In the case that this fails we switch to the max size selection option which we imple-
ment on the Pi in the following way:
Select Largest Box
areas = [(x2-x1)*(y2-y1) for (x1,y1,x2,y2,_) in boxes]
idx
= int(np.argmax(areas))
x1,y1,x2,y2,_ = boxes[idx]
We then rescale to full resolution and compute center and area:
Compute Center & Area
scale_x, scale_y = 1280/320, 960/240
sx1, sy1 = int(x1*scale_x), int(y1*scale_y)
sx2, sy2 = int(x2*scale_x), int(y2*scale_y)
x_center = (sx1 + sx2) // 2
area
= (sx2 - sx1)*(sy2 - sy1)
52
8.5.5
ESP32 Person following integration
The ESP32 receives only two values over UDP—the horizontal center of the person’s
bounding box (xCam) and the box area (areaCam). These are parsed and converted into
set-points for the inner (heading) and outer (position) control loops:
UDP Packet Parsing
int pktsz = udp.parsePacket();
if (pktsz) {
int n = udp.read(udpBuf, sizeof(udpBuf) - 1);
if (n > 0) {
udpBuf[n] = '\0';
xCam
= atoi(strtok(udpBuf, " "));
areaCam = atof(strtok(nullptr, " "));
}
}
Here, udp.parsePacket() checks for a waiting datagram, udp.read(...) reads it into
udpBuf, and strtok/atoi/atof extract the two numeric values.
Heading set-point (inner loop)
First we recenter the pixel coordinate so that zero
means straight-ahead, then convert from pixels to radians using our calibrated focal length
(32.667 px):
Pixel-to-Yaw Conversion
int
xCamCentered = xCam - 640;
float deltaYaw
= -((xCamCentered / 32.667f) * (PI/180.0f));
h_webDesired
= rotpos + deltaYaw;
xCamCentered shifts the origin to the frame centre (640 px). Dividing by 32.667 converts
to degrees, multiplying by π/180 yields radians, and the sign-flip matches the controller
convention. Finally, we add this offset to the robot’s current rotation (rotpos) to form
the heading set-point.
Position set-point (outer loop)
Next, we normalize the detected box area and map
it into a desired follow-distance offset:
Area-to-Position Conversion
float areapercent = areaCam / 1228800.0f;
g_webDesired
= posEst + (areapercent - 0.6f) * 200.0f;
Here 1228800 = 1280 × 960 px2 is the maximum thumbnail area; subtracting 0.6 makes
60
These two set-points, h webDesired and g webDesired, feed directly into the inner
and outer PID controllers to steer and balance the robot toward the tracked person.
8.5.6
Custom Model
Fine-tuning the pre-trained model allows the robot to perform better in person following
on campus because:
53
1. The custom dataset is captured from the robot camera’s angle
2. Detectable object classes is limited to one (person only)
3. Requires less data and computational resources compared to training from scratch
Hence, we collected 250 images from the robot’s camera at different orientations. LabelMe,
an open source image annotation tool is used to annotate the images. The annotated
images are then uploaded onto Roboflow for data augmentation before converting into
YOLOv8 dataset format. From evaluating the pre-trained and custom model using the
custom dataset’s test set, we see a 12% increase in accuracy using the custom model.
8.6
Controller Communication
The system includes two ESPs that communicate with each other using a peer-to-peer
protocol. This separates the user interface from the real time controller. This connection
is implemented using ESP-NOW.
8.6.1
ESP-NOW
ESP-NOW is a wireless, connectionless protocol from Espressif, that allows the exchange
of small packets directly by MAC address. This has a latency lower than one millisec-
ond. The minimization of the latency is of outmost importance, since fast and reliable
communication of the inputs to the controller should be achieved, in order to respond
quickly to the commands. This is the reason ESP-NOW was chosen, as a communication
method.
8.6.2
Alternatives Considered
Before the ESP-NOW implementation, other alternatives were considered and evaluated.
Using Wi-Fi (TCP/UDP) was considered, since it allows for an easy integration with
apps. However, this method introduces a non-negligible latency of around 10 to 100 mil-
liseconds latency, so it was rejected.
Another method considered was Bluetooth (BLE). The positive aspect is that it would
be built in into most smartphones, and it would be energy efficient, since it required low
power. Nevertheless, again it would add latency due to connection intervals. Moreover,
unpredictable delays could be introduced, because of the GATT protocol. Therefore, this
method was rejected as well.
The critical factor for this decision was the latency.
8.6.3
Data Processing
Both person following mode and manual remote control mode sends deltaYaw and deltaPitch
values to the controller ESP32.
54
Person Following Mode
int
xCamCentered = xCam - 640;
float deltaYaw
= -((xCamCentered / 32.667f) * (PI/180.0f));
float deltaPitch
= (areapercent - 0.6f) * 200.0f;
Manual Remote Control Mode
float deltaYaw
= joystick.deltaY;
float deltaPitch
= joystick.deltaX;
For both modes, the interface ESP32 constantly waits for any UDP packets containing
values needed for data processing. Once the packet is received, it will be sent to the
controller ESP32 via ESP-NOW. For example:
Interface ESP32 Processing
void handleJoystick(const String& cmd) {
memset(&txPkt, 0, sizeof(txPkt));
txPkt.isSet = false;
strncpy(txPkt.act, cmd.c_str(), sizeof(txPkt.act)-1);
txPkt.val = 0;
esp_err_t result = esp_now_send(slaveMac, (const uint8_t *)&txPkt,
sizeof(txPkt));
};
55
9
Evaluation
9.1
Finances
Table 9.1: Cost Breakdown (GBP)
Component
Qty
Unit (£)
Total (£)
Used?
Breadboard (MB-102)
3
6.37
19.11
Most of them
Operational Amplifier (TLV272I)
6
0.89
5.34
Most of them
Electret Microphone Module
1
0.76
0.76
No
Mic. Module & Audio AMP
1
4.55
4.55
Yes
PAM8403 Class-D Stereo Amplifier
1
3.19
3.19
Yes
Speaker (8 , 0.5 W)
1
1.55
1.55
Yes
Grand Total
34.5
Remaining Budget (of £60)
25.5
The given budget was £60. The remaining amount was £25.5, which indicates that
57.5 percent of the budget was used. This highlights successful financial planning which
allowed for all the required components to be bought and allowed for a contingency, just
in case of components being burnt and additional ones are needed.
Extra care was taken, when wiring components, in order to prevent burnt components
and damages. Therefore, we didn’t need to buy more components to replace any of the
ones we already had.
This indicates financial responsibility and minimizes the environmental footprint. The
cost distribution can be shown in figure 35,
Figure 35: Finances Pie Chart
56
9.2
Review of Product
We evaluated the final balance-bot against its core requirements and benchmarks men-
tioned in section 2.1. The rover was able to meet all the requirements and these results
are summarized below:
• Balance and Stability
– Stationary: remained upright > 5 min without support (benchmark: 5 min).
– Straight–line driving: maintained balance for entire test period (∼ 5min) min
at 0.5m/s (benchmark: 2 min).
– Turning: stable at up to 60ř/s for full battery cycle (benchmark: full battery).
– Disturbance recovery:
regained upright posture within 3s of lateral push
(benchmark: 5s).
• Control Performance
– Tilt recovery from 10ř disturbance in 2s, with an overshoot 3s. (benchmarks:
5s, 5s).
– Remote-control latency: 85ms round-trip at 5m (benchmark: < 100ms).
• Person Following
– Maintained following distance within 1m ± 0.4m (benchmark: 0.5m).
– Detection accuracy 92% over 5m runs (benchmark: ≥ 90%).
– End-to-end person-tracking latency 48ms (benchmark: ≤ 50ms).
• User Interface & Power Display
– Mode-switch time 180ms (benchmark: 200ms); UI refresh 35Hz (benchmark:
≥ 30Hz).
– SoC error ≤ 4% (benchmark: ≤ 5%); update rate 1.2Hz (benchmark: ≥ 1Hz).
• Head Unit and Mechanical Design
– Field-of-view ±32ř yaw (benchmark: ±30ř).
– 3D-printed camera mount tolerated repeated swaps without wear.
• Overall Assescsment
– All six core requirements met or exceeded.
– Non-technical constraints (budget, timeline, accessibility, etc.) were success-
fully satisfied.
– Minor oscillations in outer-loop speed control noted; earmarked for future
refinement.
These results demonstrate that the balance-bot fulfils its functional and performance
targets, with room for incremental improvements as outlined in Section 9.3. The robot
succesfully works and can be used for its determined purpose.
57
9.3
Future Work
While the balance-bot meets the core requirements of the project, several areas are iden-
tified for future improvements below:
• Energy Efficiency Optimization:
– Implement regenerative braking on the wheel motors and optimize sleep-mode
transitions in the ESP32 to extend battery life.
– Introduce dynamic power scaling of Raspberry Pi based on the workload
• Advanced Navigation & Obstacle Detection:
– Integrate SLAM (e.g. RTAB-Map) with LiDAR or stereo vision for mapping
and path planning.
– Add real-time obstacle detection using ultrasonic detectors to enable safe au-
tonomous travel.
• Adjustable Head-Unit Camera Mount:
– Redesign the 3D-printed holder with a quick-adjust tilt mechanism such as a
ratchet or detent to accommodate for different heights and viewing angles.
• User Experience & Accessibility Enhancements:
– Conduct formal user studies to refine the mobile and on-robot UIs, incorporate
multi-language support and adjustable text sizes.
– Add voice commands, audio prompts, and haptic feedback to improve acces-
sibility.
Reflecting on the team’s organisation and workflow, several approaches, if adapted
may have accelerated development or improved robustness:
• Team Structure & Communication:
– Hold daily stand-ups instead of weekly meetings to identify issues earlier and
align priorities more rapidly.
– Maintain a shared risk register with assigned owners to track and mitigate
technical and schedule risks proactively.
• Role Allocation & Cross-Training:
– Pair hardware and software engineers early in each module’s development to
ensure mutual understanding of interfaces and requirements.
– Cross-train team members on adjacent modules to enable quick back-up, smoother
hand-offs and ensure everyone in the team develops a deep understanding of
each module.
58
9.4
Acknowledgments
We would like to acknowledge Dr. Abd Al Rahman Ebayyeh, Dr. Christos Papavassiliou,
and Dr. Philip Clemow for their insightful feedback and guidance, and the lab technicians,
Ms. May and Mr. Vic, for their invaluable technical support.
The work of this design project was supported by the Department of Electrical and
Electronic Engineering of Imperial College London and was implemented at the depart-
ment’s lab.
We would like to also acknowledge that the completion of this group project was done
fairly and evenly by all team members, which highlights the equal distribution of the
workload.
59
References
[1]
Edward Stott. balance-robot: Balancing two-wheeled robot repository. https : / /
github.com/edstott/EE2Project/tree/main/balance-robot. Accessed: 2025-
06-16. 2025.
[2]
Mohammad Mahdi Azimi and Hamid Reza Koofigar. “Model Predictive Control
for a Two Wheeled Self Balancing Robot”. In: 2013 First RSI/ISM International
Conference on Robotics and Mechatronics (ICRoM). IEEE, 2013.
[3]
Wenyi Liu, Yunfan Ren, and Fu Zhang. “Integrated Planning and Control for Quadro-
tor Navigation in Presence of Suddenly Appearing Objects and Disturbances”. In:
IEEE Robotics and Automation Letters (2023).
[4]
Zifan Xu et al. “APPLR: Adaptive Planner Parameter Learning from Reinforce-
ment”. In: Proceedings of the Conference on Adaptive and Learning Agents. ICRA.
2021. url: https://arxiv.org/abs/2303.16106.
[5]
Raspberry Pi Foundation. Camera Module 2 Mechanical Drawing. https://datasheets.
raspberrypi.com/camera/camera- module- 2- mechanical- drawing.pdf. Ac-
cessed: 2025-06-16. 2020.
[6]
GrabCAD Community. Raspberry Pi Camera Module 4. https://grabcad.com/
library/raspberry-pi-camera-4. Accessed: 2025-06-16. 2023.
[7]
Product Specification: SubC 2000 mAh 1.2 V. Datasheet for SKU 2668, discharge
curve at 15 A. Battery Manufacturer. 2024. url: ./datasheet/battery.pdf.
[8]
Electronics Tutorials. Op Amp Circuits: The Differential Amplifier. https://www.
electronics-tutorials.ws/opamp/opamp_5.html. Accessed: 14 June 2025. n.d.
[9]
Texas Instruments Incorporated. TLV271, TLV272, TLV274 Family of 550-µA/Ch,
3-MHz, Rail-to-Rail Output Operational Amplifiers Data Sheet. https://www.ti.
com/lit/ds/symlink/tlv271.pdf. SLOS351E, revised November 2016. 2016.
60
