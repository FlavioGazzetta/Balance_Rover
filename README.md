\documentclass[12pt,a4paper]{article}
\usepackage{textcomp} 
\usepackage{setspace}
\usepackage{graphicx}
\usepackage{xcolor}
\usepackage[margin=1in]{geometry}
\usepackage{lmodern}
% For controlling vertical space
\usepackage{setspace}
\usepackage{amsmath}
\usepackage{tcolorbox}
\tcbuselibrary{skins,breakable}     % for rounded, breakable boxes
\usepackage{minted}
\usepackage{array}  
\usepackage{siunitx} % for \degree
\usepackage{enumitem}
\usepackage{booktabs}
\usepackage{subcaption}
% in your preamble
\usepackage{graphicx}
\usepackage{float}              % <-- provides the [H] placement
\usepackage[section]{placeins}  % <-- auto-inserts \FloatBarrier at each \section
% (optional tweaks if LaTeX still complains about “too many floats”)
\renewcommand{\topfraction}{0.9}
\renewcommand{\textfraction}{0.1}
\renewcommand{\floatpagefraction}{0.8}
% … your other \usepackage calls …
\usepackage[
  backend=biber,         % if you really want to continue using bibtex, change this to backend=bibtex
  style=numeric-comp,    % [1], [2–4], etc., compressed numeric style
  sorting=none           % bibliography in order of citation
]{biblatex}
\usepackage{chngcntr}
\usepackage[table]{xcolor}   % loads xcolor with table support
\usepackage{tabularx}
\setlength{\arrayrulewidth}{0.3pt}    % thinner lines
\arrayrulecolor{gray!50}  
\counterwithin{table}{section}

% 2) point to your .bib file
\addbibresource{reference/refs.bib}

\usepackage[colorlinks=true,linkcolor=blue]{hyperref}

\usepackage{listings}
\lstset{
  language=C,
  basicstyle=\ttfamily\small,
  keywordstyle=\bfseries,
  commentstyle=\itshape\color{gray},
  numbers=left,
  numberstyle=\tiny,
  stepnumber=1,
  numbersep=5pt,
  frame=single,
  breaklines=true,
  breakatwhitespace=true,
  captionpos=b
}
\newtcolorbox{CodeBox}[2][]{%
  enhanced,
  breakable,
  colback=blue!5,        % pale blue bg
  colframe=blue!50,      % deeper blue frame
  arc=2mm,               % rounded corners
  boxrule=0.5pt,         % border thickness
  left=1mm,right=1mm,top=1mm,bottom=1mm,
  title=#2,              % the caption
  fonttitle=\bfseries,
  #1                     % allow overrides
}




\begin{document}

%========================
% Title Page
%========================
\begin{titlepage}
    \includegraphics[height=2cm]{images/logo/ImperialLogo.png}\par
  \vspace{1cm}
  
  
  \begin{center}
  % Document title
  {\Huge\bfseries Balance Robot Project\par}
  \vspace{0.5cm}
  
  % Subtitle / course info
  {\Large Electronics Design Project 2\par}

  
\url{https://github.com/FlavioGazzetta/Balance_Rover.git}

  \end{center}

  
\vspace{1cm}
% ——— Author block ———
\begin{center}
  {\Large\bfseries Authors}\vspace{0.5em}

  Evangelia (Lia) Kommata (lk823),\ 
  Flavio Gazzetta (fg723),\ 
  Nabiha Saqib (ns3323),\ 
  Shenghong Liu (sl4223),\ 
  Zecheng Zhu (zz4723)
\end{center}

  
  \vspace{0.5cm}

  
  % Logo or illustration
  \begin{center}
    \includegraphics[width=0.65\textwidth]{cuteLogo.png}
  \end{center}

  \vfill
  
  % Optional wordcount line
  \immediate\write18{texcount -1 -sum=1 \jobname.tex > \jobname.wordcount}
\newread\wordcountfile
\openin\wordcountfile=\jobname.wordcount
\read\wordcountfile to \wordcounttext
\closein\wordcountfile
\newcommand{\wordcount}{\wordcounttext}

  {\small Word count: 9870 \wordcount
 \rule{2cm}{0.4pt}\par}
  
  \vspace{0.5cm}
  % Date at bottom
  {\small \today\par}
\end{titlepage}

%========================
% Table of Contents
%========================
\tableofcontents
\clearpage

%========================
% Introduction
%========================
\section{Introduction}


%========================
% Abstract
%========================
\subsection{Abstract}
Campus exploration can be enhanced by the use of autonomous robots that will elevate visitor engagement, by following them around, providing useful information, and allowing user interaction. The aim of this project was to build a two-wheel self-balancing "Campus Tour Guide" robot, that manages to maintain stability, while following a designated person through computer vision, or alternatively while being remotely controlled through an app. The robot allows users to speak to the app, ask questions, and receive answers through the robot's speaker and displays additional information about the weather through the communication between robot and the APP. The project is organized into six main sections, according to the team's division of work into modular elements. These include stability control, manual remote control, battery health display, weather display, chat mode, and person following using computer vision. The submodules were integrated on a supplied chassis. Testing after integration confirmed that all general and module-specific requirements are met, within budget. Thus, the development of a cost-effective tour guide robot is viable. Future work will explore energy efficiency optimization and SLAM-based navigation in combination with dynamic path planning.  
\clearpage
 
%========================
% Planning
%========================
\subsection{Background}

Balancing robots are canonical examples of \textit{nonlinear, underactuated systems} commonly used in control systems research and education. Due to their inherent instability, they provide an ideal platform for evaluating real‑time feedback algorithms, sensor fusion techniques, and embedded implementations. Their dynamics closely resemble those of an inverted pendulum, a classic benchmark in control theory.

\medskip



This project builds on this foundation by integrating complex interactions, including person‑following via computer vision and user interaction through voice chat, into a balancing robot. It demonstrates how classical control principles can be extended towards autonomous, intelligent, and interactive robotic systems.

\medskip
\begin{comment}
\noindent\textbf{Project Aim:}  
The primary aim of this project was to design and implement a two‑wheel self‑balancing "Campus Tour Guide" robot capable of maintaining upright stability while autonomously following a designated user via computer vision or being manually controlled through a mobile app—including voice‑based interaction and live weather display—within a low‑cost, modular, and accessible framework suitable for educational demonstration.

\medskip

By achieving this, we aim to showcase the real‑world applicability and synergy of classical control, embedded systems, computer vision, and human‑machine interaction in a cohesive, scalable platform.
\end{comment}

\subsection{Product Design}
\label{sec:product_design}

The design of the "Campus Tour Guide" balance-bot was shaped by principles of modularity, user interaction, and mechanical robustness, while ensuring all electronic and software components were integrated in a streamlined fashion.

\subsubsection{Chassis and Structural Layout}
The robot chassis was provided as a base frame, onto which all electronic modules were mounted. The design emphasized a low centre of gravity, placing batteries and the motor driver PCB at the bottom to aid balance. Above this base, the ESP32, Raspberry Pi, and associated interface electronics were installed in layers, with Velcro and 3D-printed mounts to allow modular replacement.

\subsubsection{Mechanical Modifications}
To accommodate the extended application-specific features, structural augmentations were introduced: a head unit will be 3D printed. 


\begin{comment}
\subsubsection{Modular Architecture}
A key principle of the product design was independent modularity. Each of the six major submodules—stability control, remote control, battery display, weather display, chat mode, and person-following—was developed to operate independently. Mechanical design supported this through:
\begin{itemize}[leftmargin=*]
  \item Clearly delineated zones on the chassis for each module.
  \item Standardized pin headers and voltage rails for plug-and-play board installation.
  \item Use of labelled PCB overlays and connector colour-coding to minimize integration risk.
\end{itemize}
\end{comment}

\subsubsection{Power Management Layout}
The power system was split into two isolated rails: logic (5V) and motor drive (12V), each regulated and monitored separately. The layout was carefully designed to minimize noise coupling, with analog circuits for current sensing placed away from high-frequency PWM motor lines. Decoupling capacitors and a dedicated ground plane were used to reduce ripple.

\subsubsection{Human and Robot Interaction}
A strong emphasis was placed on interaction:
\begin{itemize}[leftmargin=*]
  \item The handle allowed easy repositioning and pick-up of the robot.
  \item Visual cues, such as a five-bar battery indicator and clear menu UI, enhanced usability.
  \item Audio output and voice control supported interaction by visually impaired users.
\end{itemize}

\subsubsection{Design for Safety and Maintenance}
Safety was integrated from the design stage. The chassis was reinforced at high-impact zones, fuses protected all voltage rails, and the wheelbase was optimized for tilt-resistance. Additionally:
\begin{itemize}[leftmargin=*]
  \item Screwed terminals allowed easy part swaps without soldering.
  \item Over-current and voltage drop-off detection enabled graceful shutdown.
\end{itemize}

\subsubsection{Sustainability Considerations}
The product was designed with minimal environmental impact:
\begin{itemize}[leftmargin=*]
  \item PLA filament was used for all 3D prints, offering biodegradability.
  \item All modular units are reusable or upgradable.
  
\end{itemize}

This product design ensured the robot was not only functionally reliable and safe but also adaptable for future expansion and improvements.

\clearpage

\subsection{Project Management}
\subsubsection{Project Framework Adapted}
The Agile Framework was chosen in order to minimize integration risks and ensure rigorous project control. This consisted of one week iterative sprints, which prioritized high-risk and critical tasks. Frequent reviews and demonstration lead to continuous improvement. Cross-functional collaboration between both the hardware and software team allowed for a quicker integration.

\begin{figure}[H]
    \centering
    \includegraphics[width=0.85\linewidth]{framework.jpg}
    \caption{Agile Framework Adapted}
    \label{fig:project-framework}
\end{figure}


\subsubsection{Role Allocation}
 Tasks were allocated in the beginning of the project based on interests, knowledge, and expertise. Tasks were assigned in a way to ensure that the distribution of the work was fair.   Throughout the development, every team member was part of the integration to ensure its smoothness. 

% …

\begin{table}[H]
  \centering
  \caption{Team Responsibilities}
  \label{tab:team-responsibilities}
  \begin{tabular}{@{} l l @{}}
    \toprule
    \textbf{Module}              & \textbf{Members}                           \\
    \midrule
    Main Controller              & Flavio, Lia, Nabiha                       \\
    LQR Controller               & Zecheng                                   \\
    Battery Health Display       & Lia, Nabiha                               \\
    Weather Mode Data Display    & Zecheng                                   \\
    Chat AI Mode                 & Shenghong                                 \\
    Person Following Mode        & Flavio                                    \\
    User Interface               & Shenghong                                 \\
    Head Unit                    & Nabiha                                    \\
    Integration                   & Flavio, Lia, Nabiha, Shenghong, Zecheng   \\
    \bottomrule
  \end{tabular}
\end{table}

 Constant communication was maintained through out the whole project between different modules and each member of the team would help the other members when issues were present, even if they were working on different modules. This ensured high success and minimal integration issues.  


\subsubsection{Timeline}
A Gantt chart (figure ~\ref{fig:timeline}) was created to certify efficient allocation of temporal resources. Schedule successfully remained mostly unchanged throughout the whole project. 
\begin{figure}[H]
    \centering
    \includegraphics[width=1\linewidth]{Timeline.jpg}
    \caption{Gantt Chart}
    \label{fig:timeline}
\end{figure}


\subsubsection{Weekly Plan}
Every Friday online meetings were held to examine individual progression in previous tasks and assign future deliverables. Rigorous documentation was kept from the start to track progress, which was kept in a shared folder. Every Tuesday in person meetings were held to discuss the tasks of the week. 


\subsubsection{Risk and Contingencies}
\label{sec:risks}

To ensure proper functionality of the balance-bot project, key risks were identified and sorted into four categories. These categories define the mitigating actions and fall-backs:

\begin{itemize}[leftmargin=*]
  \item \textbf{Hardware \& Sensor Risks:}
    \begin{itemize}[nosep]
      \item IMU Failure and Noise: Keep a spare MPU-6050 on hand; verify I²C wiring and add runtime self-tests.  
      \item Overheating of Stepper Motor and Stalling: Cap maximum step rate in firmware, stock an extra driver board, monitor stall flags and shut down gracefully.  
      \item Power and Battery Drop: Use a regulated bench supply for early tests and validate at least 30 min runtime on batteries. Implement low-voltage cutoff in firmware.  
    \end{itemize}

  \item \textbf{Software and Timing Risks:}
    \begin{itemize}[nosep]
    \item Overloading Control Loop:
      Enforce a minimum 10 µs gap between stepper driver pulses; profile and optimise ISR routines; degrade or suspend non-critical tasks under high CPU load.
  \end{itemize}

  \item \textbf{Project and Scheduling Risks:}
    \begin{itemize}[nosep]
      \item Part Delivery Delays: Order critical components early and identify alternative suppliers in-case of unavailability. Begin algorithm development in simulation while awaiting hardware.  
      \item Scope Creep: Prioritize core requirements before extensible module.  
      \item Communication Gaps: Hold weekly team meetings and maintain concise, up-to-date documentation in a shared repository.  
    \end{itemize}

  \item \textbf{Safety and Contingencies:}
    \begin{itemize}[nosep]
      \item Robot Tipping and Damage: Perform initial tests in a contained arena and using tethered power supply unit for early trials.  
      \item Electrical Shorts: Double-check wiring before each test and using the current-limited bench supply first.
    \end{itemize}
\end{itemize}
\clearpage

\section{Requirements Capture}
\label{sec:requirements}

\subsection{Core Requirements, Adaptations, and Verification Benchmarks}
\label{sec:core_requirements}
Six core requirements were taken from the GitHub repository \cite{stott2025balance} shown below. These capture the basic requirements of the robot and show the system‐specific adaptations used.
They also outline how these requirements were modified for this application and the verification benchmark used. 


\begin{enumerate}[leftmargin=*]
  \item \textbf{Autonomous behaviour based on camera/sensors}
    \begin{itemize}[nosep]
      \item {Adaptation:} Person‐Following implemented with YOLOv8 nano over a UDP socket on the Raspberry Pi.
      \item {Verification:} Person‐following accuracy ≥ 90 % over a 5 m run.
    \end{itemize}

  \item \textbf{Balance on two wheels with CoG above axle}
    \begin{itemize}[nosep]
      \item {Adaptation:} Dual PID loops with complementary filter (c = 0.98).
      \item {Verification:} Recover from a 10° tilt in ≤ 5 s with ≤ 5 % overshoot.
    \end{itemize}

  \item \textbf{Remote‐control interface for switching between autonomous and manual drive}
    \begin{itemize}[nosep]
      \item {Adaptation:} Flutter mobile app communicating via REST API on the ESP32.
      \item {Verification:} Round‐trip command latency < 100 ms at 5 m range.
    \end{itemize}

  \item \textbf{Display power status (consumption and remaining energy)}
    \begin{itemize}[nosep]
      \item {Adaptation:} OLED and mobile UI, using a Coulomb‐count + lookup‐table hybrid method.
      \item {Verification:} State‐of‐Charge error ≤ 5 %, update rate ≥ 1 Hz.
    \end{itemize}

  \item \textbf{UI inputs/displays pertinent to demonstrator}
    \begin{itemize}[nosep]
      \item {Adaptation:} Four‐mode menu (Balance, Weather, Chat, Follow).
      \item {Verification:} Mode‐switch time ≤ 200 ms; UI refresh rate ≥ 30 Hz.
    \end{itemize}

  \item \textbf{Augment chassis with application‐specific head unit}
    \begin{itemize}[nosep]
      \item {Adaptation:} 3D‐printed head unit with 15° camera tilt and 2 % manufacturing tolerance.
      \item {Verification:} Field‐of‐view ± 30° yaw; handle lift capacity ≥ 2 kg.
    \end{itemize}
\end{enumerate}

These requirements are further expanded into the sections below. 

\subsection{Stability and Recovery Requirements}
These requirements ensure the robot maintains upright posture under various conditions.

\begin{enumerate}[leftmargin=*]
  \item \textbf{Stationary Stability}
    \begin{itemize}[nosep]
      \item {Test Method:} Power on while the robot is held upright.
      \item {Acceptance Criterion:} Remain balanced for at least 5 minutes without external support.
    \end{itemize}

  \item \textbf{Straight‐Line Stability}
    \begin{itemize}[nosep]
      \item{Test Method:} Drive the robot forwards and backwards at 0.5 m/s.
      \item {Acceptance Criterion:} Maintain balance for at least 2 minutes of continuous motion.
    \end{itemize}

  \item \textbf{Turning Stability}
    \begin{itemize}[nosep]
      \item {Test Method:} Rotate in place at up to 60°/s.
      \item {Acceptance Criterion:} Remain balanced for the full duration of the battery life.
    \end{itemize}

  \item \textbf{Disturbance Recovery}
    \begin{itemize}[nosep]
      \item {Test Method:} Deliver a brief lateral push to the robot.
      \item {Acceptance Criterion:} Regain upright balance within 5 seconds of the disturbance.
    \end{itemize}
\end{enumerate}

\subsection{Manual Remote Control Requirement}
This implements user-driver motion via controls present on the smart phone app. 

\begin{enumerate}[leftmargin=*]
  \item \textbf{Move up to 0.5 m/s and turn at up to 90°/s}
    \begin{itemize}[nosep]
      \item {Test Method:} Log wheel speed and turn rate using the onboard encoder and gyroscope.
      \item {Acceptance Criterion:} Achieve commanded speed within ± 5 % and turn rate within ± 5°/s.
    \end{itemize}

  \item \textbf{Control reaction time ≤ 100 ms at 5 m range}
    \begin{itemize}[nosep]
      \item {Test Method:} Timestamp command transmission and reception over BLE at 5 m.
      \item {Acceptance Criterion:} Round‐trip command latency remains < 100 ms over multiple trials.
    \end{itemize}
\end{enumerate}


\subsection{Extensible Module Architecture}
To support extensible feature additions and streamlined testing, software and hardware were organised into separated modules and each of the extensible module was broken down such that it could be implemented independently. Each module is summarised below, with its responsibility, and the extensibility mechanism.
\begin{enumerate}[leftmargin=*]
  \item \textbf{Weather Mode Data Display}
    \begin{itemize}[nosep]
      \item {Functionality:} Fetch local weather via API; present temperature, humidity, etc.
      \item {Performance Benchmark:} Display updates within ≤ 500 ms after request.
    \end{itemize}

  \item \textbf{Chat Mode Voice Input}
    \begin{itemize}[nosep]
      \item {Functionality:} Capture speech and speech-to-text processing
      \item \textit{Performance Benchmark:} Text processed and outputted within 500 ms of speaking.
    \end{itemize}

  \item \textbf{Chat Memory Persistence}
    \begin{itemize}[nosep]
      \item {Functionality:} Persist summaries of chat prompts and responses in a SQL database.
      \item {Performance Benchmark:} Read/write latency ≤ 100 ms.
    \end{itemize}

  \item \textbf{Battery \& Power Status Display}
    \begin{itemize}[nosep]
      \item {Functionality:} Measure current via differential amplifier; compute state-of-charge (SoC) and power.
      \item {Performance Benchmark:} Update readings every 1 s with ≤ 5 % SoC error.
    \end{itemize}

  \item \textbf{Person Following}
    \begin{itemize}[nosep]
      \item {Functionality:} Track the designated person using YOLOv8.
      \item {Performance Benchmark:} Maintain a following distance of 1 m ± 0.5 m.
    \end{itemize}
\end{enumerate}

\subsection{Non-Technical Requirements}
Beyond functionality, the project imposed organizational and usability constraints. The details of each of these non-technical requirement and how it influenced our process.

\begin{enumerate}[leftmargin=*]
  \item \textbf{Budget Constraint}
    \begin{itemize}[nosep]
      \item Description: Total cost must remain under £60.
      \item Implementation Impact: Prioritised open-source libraries; 3D-printed head in PLA to reduce cost.
    \end{itemize}

  \item \textbf{Development Timeline}
    \begin{itemize}[nosep]
      \item Description: Interim demo at week 6; final report at week 12.
      \item Implementation Impact: Adopted weekly Agile sprints; automated CI checks on each push.
    \end{itemize}

  \item \textbf{Documentation \& Traceability}
    \begin{itemize}[nosep]
      \item Description: Maintain clear documentation for future teams.
      \item Implementation Impact: Used GitHub for code; refreshed project Wiki after every sprint.
    \end{itemize}

  \item \textbf{Team Collaboration}
    \begin{itemize}[nosep]
      \item Description: Cross-disciplinary work between hardware and software groups.
      \item Implementation Impact: Bi-weekly integration meetings; shared GitHub issues and pull-requests.
    \end{itemize}

  \item \textbf{Accessibility}
    \begin{itemize}[nosep]
      \item Description: UI must be usable by non-technical stakeholders (e.g. visitors).
      \item Implementation Impact: High-contrast UI theme; simplified menu labels; optional voice prompts.
    \end{itemize}

  \item \textbf{Equity and Inclusion}
    \begin{itemize}[nosep]
      \item Description: Ensure the system is usable by visually impaired users.
      \item Implementation Impact: Audio feedback; tactile buttons; voice control support.
    \end{itemize}

  \item \textbf{Educational Outreach}
    \begin{itemize}[nosep]
      \item Description: The system should serve as a teaching tool for students and novices.
      \item Implementation Impact: Included interactive UI overlays and a talk-back feature.
    \end{itemize}

  \item \textbf{Environmental Sustainability}
    \begin{itemize}[nosep]
      \item Description: Minimise environmental impact and energy consumption during use.
      \item Implementation Impact: Low-power sleep modes; uses minimal components.
    \end{itemize}

  \item \textbf{Safety \& Compliance}
    \begin{itemize}[nosep]
      \item Description: Electrical and mechanical safety for public demonstrations.
      \item Implementation Impact: Added emergency-stop button; over-current protection; CE-compliant wiring.
    \end{itemize}
\end{enumerate}

\clearpage

\section{Modelling}
%%========================
%% Modelling
%%========================
The dynamics of the robot are described by a mathematical model to facilitate the development of an efficient control system. This section derives the equation of motion for a two-wheeled inverted pendulum with stepper motor actuation.

\subsubsection{Stepper Motor Model}

The robot uses two stepper motors for wheel actuation. Unlike DC motors, steppers provide discrete position control through step pulses. For control system design, we model the stepper motor dynamics in continuous time with the following assumptions:

\begin{itemize}
\item High step resolution (microstepping) allows quasi-continuous motion
\item Step frequency is proportional to desired angular velocity
\item Motor torque is approximately constant within operating speed range
\end{itemize}

The simplified stepper motor relationship is:
\begin{equation}
\tau_m = K_s \cdot f_{step}
\label{eq:stepper_torque}
\end{equation}

where $K_s$ is the stepper torque constant (Nm·s) and $f_{step}$ is the step frequency (Hz), which relates to the control input.

For the wheel dynamics, we replace the DC motor equations with a direct torque input model:
\begin{equation}
H_{fR} = \frac{1}{r}\left[(I_w + I_m)\ddot{\theta_w} - \tau_m\right]
\label{eq:stepper_friction_force}
\end{equation}

\subsubsection{Dynamic model for a two wheeled inverted pendulum}

\textbf{Variable Definitions}
\begin{table}[h]
\centering
\begin{tabular}{|c|l|c|}
\hline
\textbf{Variable} & \textbf{Description} & \textbf{Units} \\
\hline
$x$ & Horizontal position of robot & m \\
$\phi$ & Pendulum angle from vertical & rad \\
$\theta_w$ & Wheel angle & rad \\
$M_w$ & Mass of each wheel & kg \\
$M_p$ & Mass of pendulum/chassis & kg \\
$I_w$ & Moment of inertia of wheel & kg·m² \\
$I_p$ & Moment of inertia of pendulum about center of mass & kg·m² \\
$I_m$ & Motor inertia reflected to wheel & kg·m² \\
$l$ & Distance from wheel axle to pendulum center of mass & m \\
$r$ & Wheel radius & m \\
$g$ & Gravitational acceleration & m/s² \\
$H_L, H_R$ & Horizontal forces from left and right wheels & N \\
$\tau_m$ & Applied motor torque & Nm \\
$K_s$ & Stepper torque constant & Nm·s \\
\hline
\end{tabular}
\caption{Two-Wheeled Inverted Pendulum Parameters}
\label{table:pendulum_params}
\end{table}

The robot consists of two main components: wheels and an inverted pendulum chassis. We analyze each separately then combine them.

\textbf{Wheel Dynamics}

For each wheel actuated by a stepper motor, the dynamics can be described by Equation~(3).  
Using the kinematic constraint $\theta_w = x/r$ and its differentiation, the dynamics of the wheel can be rewritten as:


\begin{equation}
2\left(M_w + \frac{I_w + I_m}{r^2}\right) \ddot{x} = \frac{2\tau_m}{r} - (H_L + H_R)
\label{eq:combined_wheels}
\end{equation}

\textbf{Pendulum Dynamics}
For the inverted pendulum chassis, applying Newton's laws in horizontal and rotational directions:

Horizontal force balance for inertial force, centrifugal force and tangential acceleration forces.
\begin{equation}
(H_L + H_R) = M_p \ddot{x} - M_p l \sin\phi \dot{\phi}^2 + M_p l \cos\phi \ddot{\phi}
\label{eq:horizontal_forces}
\end{equation}

Moment balance about center of mass:
\begin{equation}
I_p \ddot{\phi} = \frac{2\tau_m}{r} - l(H_L + H_R) \sin\phi
\label{eq:pendulum_moment}
\end{equation}

\textbf{Linearization}
For small angles from vertical ($\phi \approx 0$), we use the approximations:
$\cos\phi \approx 1$, $\sin\phi \approx \phi$, $\dot{\phi}^2 \approx 0$

This yields the linearized equations:
\begin{equation}
(I_p + M_p l^2) \ddot{\phi} + M_p l \ddot{x} = -M_p g l \phi + \frac{2\tau_m}{r}
\label{eq:linearized_pendulum}
\end{equation}

\begin{equation}
\left(2M_w + \frac{2I_w + 2I_m}{r^2} + M_p\right) \ddot{x} + M_p l \ddot{\phi} = \frac{2\tau_m}{r}
\label{eq:linearized_chassis}
\end{equation}

\textbf{State Space Representation}
Solving the coupled equations above for $\ddot{x}$ and $\ddot{\phi}$, the final state space model is:

\begin{equation}
\begin{bmatrix}
\dot{x} \\
\ddot{x} \\
\dot{\phi} \\
\ddot{\phi}
\end{bmatrix} = 
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & \frac{M_p^2 g l^2}{\Delta} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & -\frac{M_p g l M_T}{\Delta} & 0
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\phi \\
\dot{\phi}
\end{bmatrix} + 
\begin{bmatrix}
0 \\
\frac{2 M_p l}{\Delta r} \\
0 \\
-\frac{2 M_T}{\Delta r}
\end{bmatrix} \tau_m
\label{eq:final_state_space}
\end{equation}

where:
\begin{align}
M_T &= 2M_w + \frac{2I_w + 2I_m}{r^2} + M_p \\
\Delta &= M_T(I_p + M_p l^2) - M_p^2 l^2
\end{align}

This model assumes no wheel slip, continuous ground contact, and that the stepper motors can provide the required torque $\tau_m$ as the control input.

\clearpage
% MPC/LQR
\subsection{Model Predictive Control Approach}


Inspired by state-of-the-art control methods~\cite{azimi2013mpc, liu2023integrated}, we aimed to implement a Model Predictive Control (MPC) strategy for a two-wheel balancing robot. The  cost function used for discrete-time system dynamics optimization is given by:

\[
J(X, U) = \sum_{k=0}^{N-1} \left( x_k^\top Q x_k + u_k^\top R u_k \right) + x_N^\top P x_N,
\]

where \(Q\), \(R\), and \(P\) represent weighting matrices for state regulation. 

At each sampling instant, an optimal control sequence is computed based on real-time state estimation, and only the first control input is applied, with the process repeated in a receding-horizon fashion. However, due to the computational demands of solving the optimization problem in real time on embedded hardware, we replaced the full MPC formulation with its linearized counterpart—the Linear Quadratic Regulator (LQR). The LQR controller uses a pre-derived optimal state-feedback gain matrix to stabilize the system.

While LQR present better balancing, experimental results revealed limited robustness in the outer position loop, especially under external disturbances or model mismatches. To address this, we drew inspiration from Xu et al.~\cite{xu2023applr}, who proposed an reinforcement learning approach to improve motion planning performance for mobile robot.

A simulation-based training framework was designed using the Twin Delayed Deep Deterministic Policy Gradient (TD3) algorithm, leveraging the BARN dataset with 300 diverse navigation environments to train this single agent under Gazebo. The sparse reward function was crafted to penalize tracking deviations and penalize falls, encouraging robust trajectory-following behaviour. However, due to limited computational resources on a personal laptop, sim-to-real transfer was not fully executed despite several rounds of reward tuning and convergence in simulation. As a result, we ultimately adopted a robust PID controller for real-world deployment, prioritizing stability and real-time responsiveness.

\begin{figure}[H]
  \centering
  % first image, 80% of text width
  \begin{subfigure}[b]{0.7\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/mpc/lqr.png}
    \caption{LQR Controller}
    \label{fig:lqr}
  \end{subfigure}

  \vspace{1em} % small vertical gap

  % second image
  \begin{subfigure}[b]{0.7\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/mpc/simw.png}
    \caption{Simulation Environment in Gazebo}
    \label{fig:sim_world}
  \end{subfigure}

  \vspace{1em}

  % third image
  \begin{subfigure}[b]{0.7\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/mpc/train.jpg}
    \caption{Training Performance Curve}
    \label{fig:training_result}
  \end{subfigure}

  \caption{LQR Controller, Simulated Environment, and Learning Performance.}
  \label{fig:stacked_fig}
\end{figure}
\clearpage


\section{Control System}
% \clearpage
A control system is designed to balance the rover on two wheels, with the centre of gravity above the rotation axis of the wheels. Additionally, disturbance recovery is also implemented. 
Stability is separated into three requirements:
\begin{itemize}
    \item While standing still - static stability 
    \item While moving forward and backwards 
    \item While rotating
\end{itemize}


\subsection{Design Overview}
The controller is designed with dual cascading loop structure to fulfil the balancing requirements mentioned above. The inner, faster loop consists of two PID controllers, responsible for stability when standing still and when rotating respectively. It uses the processed signals from the MPU6050 (accelerometer and gyroscope) as inputs. 
The outer, slower loop is a position controller, which uses the average velocity values as input. It is responsible for the forward and backward movement of the robot.

Figure~\ref{fig:general_structure} illustrates this structure. 

\begin{figure}[!htbp]
    \centering
    \includegraphics[width=1.0\linewidth]{GeneralApproach.png}
    \caption{Control System Structure}
    \label{fig:general_structure}
\end{figure}


\subsection{Sensors and Signal Processing}
\label{sec:sensors-sig-processing}

The balancing controller relies on tilt measurement from the MPU6050, the combined accelerometer and gyroscope sensor. 

\begin{itemize}
    \item \textbf{3 Axis Accelerometer:} Measures the vector sum of gravitational and inertial accelerations, and therefore gives the tilt of the robot. However, it cannot distinguish between a gravitational force and a force due to acceleration, and hence would introduce a transient error when the robot moves longitudinally.
    \item \textbf{3 Axis Gyroscope:} Measures differential or the rate of change of the tilt angle. However, the output of the sensor has a small error which accumulates when integrating, leading to drift.
\end{itemize}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.35\linewidth]{MOU6050.jpg}
    \caption{MPU6050}
    \label{fig:MPU}
\end{figure}

These errors are corrected using a complimentary filter (eq~\ref{eq:complementary_filter}), which produces a stable, drift-free estimate of tilt. 
The complementary filter achieves this by performing a summation of a low pass filtered accelerometer tilt measurement and a high pass filtered gyroscope tilt measurement. 

\begin{equation}
\theta_{n}
= (1 - c)\,\theta_{a,n}
\;+\;
c\Bigl(\theta_{n-1} + \dot{\theta}_{g,n}\,\Delta t\Bigr)
\label{eq:complementary_filter}
\end{equation}

Here, 
\(\theta_{a,n}\) is the accelerometer‐derived tilt at timestep \(n\), 
\(\dot{\theta}_{g,n} = \tfrac{d\theta_g}{dt}\) is the gyroscope’s angular rate, 
\(\Delta t\) is the sampling interval, and 
\(c\in[0,1]\) is the filter constant. 

The filter constant is chosen to be \(c=0.98\), to emphasize the gyroscope dynamics, allowing accurate measurement for rapid tilt variations and limiting accumulated error bias.


\subsection{Inner Loop PID controller}
The inner loop repeats every 20 ms and implements two separate proportional integral derivative (PID) controllers. The controller's purpose is to minimize the error \(e(t)\) which is calculated as the difference between the robot's current angle and the set-point while avoiding a large overshoot. 

The PID controller sums the error multiplied by a constant \(K_p\), the derivative of the error multiplied by a constant \(K_d\), and the integral of the error multiplied by a constant \( K_i\) (Equation~\ref{eq:pid_controller}).
This produces the control output \(u(t)\), which is the velocity command needed to drive the system output toward the desired set-point.

\begin{equation}
u(t) = K_p\,e(t) \;+\; K_d\,\frac{\mathrm{d}e(t)}{\mathrm{d}t} \;+\; K_i \int_{0}^{t} e(\tau)\,\mathrm{d}\tau
\label{eq:pid_controller}
\end{equation}

In the PID controller: 
\begin{itemize}
    \item linear term indicates the primary corrective torque
    \item derivative term indicates the damping movement and is preventive of oscillations
    \item integral term eliminates the static error, which is the steady-state offset
\end{itemize}

\begin{CodeBox}{Inner-loop error calculation}
\begin{minted}[fontsize=\small,linenos=false]{c++}
err_inner = (ref + tiltSP) - theta;
integral  += err_inner * dt;
deriv      = -gyro_y;
\end{minted}
\end{CodeBox}



\subsubsection{Static PID}
The parameter values chosen after tuning, shown in Section~\ref{sec:sensors-sig-processing}, are  as follows:

\[
K_p = 1000,\quad K_d = 200,\quad K_i = 1.
\]


These values imply a control strategy where the proportional term dominates, ensuring rapid correction of any deviation in the error. 
The derivative term is high, but has less contribution, as it provides damping to reduce overshoot and makes the controller resistant to sudden changes in the angular velocity. 
The integral gain is relatively much smaller and gradually eliminates steady‐state error. It is limited to a small value to avoid high overshoots, but still have enough static error correction to return to the desired position. 


A complementary filter, as shown in Section~\ref{sec:sensors-sig-processing} combines the accelerometer \(z\) and \(x\) axis readings and the pitch of the gyroscope to find the robot’s angular distance from the reference (\(\theta\)).


\subsubsection{Rotation PID}
The rotation PID controller is based on the same logic as the standard inner loop but uses the gyroscope roll output to track its position. It implements this by using the previous rotation angle and adding how much it has changed since the last measurement. 

\begin{CodeBox}{Rotation angle calculation}
\begin{minted}[fontsize=\small,linenos=false]{c++}  
rotpos = prev_rotpos + spinRate * dt;
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

It then uses the rotation error, the rotation errors derivative and its integral for the controller.

\begin{CodeBox}{Periodic control loop within the \texttt{INNER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
spinErr   = h_webDesired - rotpos;
spinDeriv = (spinErr - prev_spinErr) / dt;
spinIntegral += spinErr * dt;
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

As in a standard PID, each of these values are multiplied by their respective gain values and summed together.

\begin{CodeBox}{Periodic control loop within the \texttt{INNER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
float Prot = rp * spinErr;
float Drot = rd * spinDeriv;
float Irot = ri * spinIntegral;
float rotvel = Prot + Drot + Irot;
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

This controller has the following gain values:
\[
K_p = 2,\quad K_d = 0.5,\quad K_i = 1.
\]

The rotation controller can be implemented with much lower gain values because the rotation speed is considerably lower than the speed required to keep the controller stable. 

This rotvel output would then be added to the inner stability loops target speed of one wheel and subtracted from the target speed of the other wheel, implementing rotation.

\begin{CodeBox}{Periodic control loop within the \texttt{INNER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
step1.setTargetSpeedRad( uout  - rotvel );
step2.setTargetSpeedRad( uout  + rotvel );
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

\subsection{Outer Loop controller}
To ensure that the robot can hold and attain a desired position, a slower outer is loop  implemented, which repeats every 100 ms. The velocity of the rover is increased and decreased by varying a constant which is added to the reference tilt, which would lead the inner loop controller to make the rover speed up or slow down in order to prevent it from falling over. 
In order to avoid an exaggerated tilt, constraints on the maximum angle are included, by which the reference angle is varied.\\
The way this controller works is that it is separated into 3 main parts, one for when the robot is on or is very close to the desired position, one for when the robot was within a small bound of the desired position and one when it was very far away. For this we used 2 thresholds separating these stages into three stages, with the distances of 10 and 5.\\

The way the furthest distance mode works is that the loop uses an average velocity calculated over the past 5 iterations of the inner loop to see how fast it is moving. This value is then reset to 0 after every time it is used in the outer loop.


\vspace{0.5 cm}
\begin{CodeBox}{Speed averaging calculation}
\begin{minted}[fontsize=\small,linenos=false]{c++}
speedcount++;

SumSpeed += 0.5f * (sp1_meas + sp2_meas);

avgSpeed = SumSpeed / speedcount;
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}
    
It then uses this to edit the speed, and then the system works to try and achieve this specific speed. 
The direction of the rover would be determined by the sign of the position error (desired position - actual position), making it move towards the left side if it was too far right and vice versa. \\

\begin{CodeBox}{Periodic control loop at \texttt{OUTER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
if(abs(posErr) > POSERRLIMIT){
    if(posErr < 0){
        Kp = (-0.05)/(2);
    }else{
        Kp = 0.05;
    }
    tiltSP = -constrain((Kp/abs(avgSpeed)),
    ANGLE_CONSTRAINT/2, ANGLE_CONSTRAINT);
}
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

Once the position error is small enough, this passes the outer threshold, after which it uses a combination of the velocity and the magnitude of the position error to slow down faster as it gets closer to its desired position. This is done to avoid an overshoot.\\

\begin{CodeBox}{Periodic control loop between \texttt{Outer\_INTERVAL} and \texttt{INNER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
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
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

After the robot gets even closer to the desired position, it would pass the inner threshold, after which it would no longer take into account the outer loop controller. This is implemented by setting the outer loop value added to the reference equal to 0, and would simply remain stable within the desired region.

\begin{CodeBox}{Periodic control loop within the \texttt{INNER\_INTERVAL}}
\begin{minted}[fontsize=\small,linenos=false]{c++}
if(absErr < POSERRSLOWLIMIT){
    tiltSP = 0;
}
\end{minted}
\end{CodeBox}
 \vspace{0.5 cm}

One thing to note about this approach is that it causes the velocity of the rover to oscillate around the desired value as demonstrated in figure~\ref{fig:curr-velocity-variation}.

\begin{figure}[H]
    \centering
    \includegraphics[width=0.85\linewidth]{images/controller/currentVelocity.jpg}
    \caption{Plot of the velocity of the rover against time, current method}
    \label{fig:curr-velocity-variation}
\end{figure}

This method works successfully to achieve stability while moving, but the rover slows down and then accelerates repeatedly. 
\\ 

A future development to avoid this would be to make it such that the rover only initially receives a tilt. This would lead to it accelerating in the desired direction, and soon after, it will go back to a tilt close to the usual reference (slightly inclined to take into account friction and air resistance). Since it has now achieved its desired velocity, it can then keep moving at this constant velocity until it is near the desired position. After this, it receives the same magnitude of tilt as before but in the opposite direction such that decelerates and goes back to being stationary.

This method is graphically described in figure~\ref{fig:tilt-variation} which shows the tilt and figure~\ref{fig:velocity-variation} which models the velocity. 

\begin{figure}[H]
    \centering
    \includegraphics[width=0.85\linewidth]{images/controller/tiltplot.jpg}
    \caption{Plot of the tilt of the rover against time, method for future implementation}
    \label{fig:tilt-variation}
\end{figure}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.85\linewidth]{images/controller/velocityPlot.jpg}
    \caption{Plot of the velocity of the rover against time, method for future implementation}
    \label{fig:velocity-variation}
\end{figure}

\clearpage
\subsection{Parameter Tuning}
\label{parameter-tuning}
Multiple methods were employed to tune the parameter, to find the most suitable values for the PID Controller (\(K_P, K_I, K_D\)) that produce most stable behaviour. 
This was done in multiple stages: 
\begin{itemize}
    \item Simulation
    \item Loop Shaping
    \item Trial and Error
\end{itemize}

\subsubsection{Simulation}
Multi-joint dynamics with contact (MuJoCo)  Python-Tkinter simulation was developed to accelerate the tuning process. The robot was simulated as a body on two wheels, with the centre of gravity above the rotation axis of the wheels, mimicking the robot. 

Rapid trial and error was performed to tune the PID gains and balancing stability was achieved in an ideal, frictionless environment in the simulation. 

Although, the physical robot did not balance when tested with these PID gains, it provided a good starting point for loop shaping and significantly reduces the time required for hardware parameter tuning.

The robot did not balance in real word due to air resistance, surface friction, noise in the sensor, overheating of the components, vibrations in the chassis and the motors dead zone. 

\begin{figure}[H]
  \centering
  \begin{subfigure}[b]{0.37\textwidth}
    \centering
    \includegraphics[width=\textwidth]{ParamSim.png}
    \caption{Simulation Parameters Tuning}
    \label{fig:design1}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.59\textwidth}
    \centering
    \includegraphics[width=\textwidth]{Sim1.png}
    \caption{Simulation of the robot}
    \label{fig:design2}
  \end{subfigure}
  \caption{MuJoCo Simulation Trial and Error}
  \label{fig:designs_side_by_side}
\end{figure}

\subsubsection{Trial-and-Error Tuning}
\label{subsubsec:trial_error}
Balancing was achieved by tuning the parameters of a default PID controller.

On the real robot, the initial gain values were set to those found using simulation and then were iteratively adjusted.
These adjustments were made based on the behaviour of the robot observed. 

If there was an overshoot. which can be observed when the robot runs and falls over, then the proportional gain was increased to provide a larger restoring force. 
 
If the robot oscillated before falling over, then either the proportional gain was decreased or differential gain was increased to introduce damping and remove overshoot.

The integral gain was kept relatively low while tuning. This is because a reference was added to the inner loop which indicated the the right set-point for static conditions. 
This reference accounts for the offset of the centre of  mass of the robot from the central axis used to measure tilt and any misalignment between the tilt sensor and axis.
This offset was found by turning the motors off and finding the tilt angle measured when the robot naturally balances as it is gently held upright. 

A general guideline used for trial and error is displayed in table:

 \begin{table}[H]
  \centering
  \label{tab:pid_tuning_considerations}
  \begin{tabular}{@{}lccccp{6cm}@{}}
    \toprule
    \textbf{Gain} & \textbf{Rise Time} & \textbf{Overshoot} & \textbf{Settling Time} & \textbf{Steady‐State Error} \\
    \midrule
    $K_p$ & Decrease & Increase & No effect & Decrease \\
    $K_i$ & Decrease & Increase & Increase & Eliminate \\
    $K_d$ & No effect & Decrease & Decrease & No effect\\
    \bottomrule
  \end{tabular}
  \caption{Effects of PID Gains for Tuning }
\end{table}
\subsubsection{Loop-Shaping Control Design (Considered)}
\label{subsubsec:loop_shaping_considered}

Loop-shaping is a frequency-domain method that designs a controller \(C(s)\) to shape an open-loop transfer \(L(s)=C(s)G(s)\) which meets the robustness and performance targets of the robot which include the bandwidth, phase-margin, disturbance-rejection and noise attenuation. 

Although this method was not used to structure the controller for reasons mentioned later, a typical workflow followed would be:

\begin{enumerate}[label=\arabic*.]
  \item {Model identification:}  
    Obtaining a low-order model of the system from step responses or frequency sweeps.
    \[
      G(s)\approx\frac{K}{(T_1s+1)(T_2s+1)}
    \]
    
  \item {Specification definition:}  
    Choosing a desired crossover frequency \(\omega_c\), phase-margin \(\phi_m\), and sensitivity bounds (e.g.\ \(\lvert S(j\omega)\rvert<0.1\) for \(\omega<1\)\,rad/s). These values would be chosen based on the time domain requirements of our system including settling time, max overshoot, and acceptable steady-state error. 
  \item {Compensator structure:}  
    Selecting a cascaded lead/lag and roll-off sections, for example
    \[
      C(s)=K_c\;\frac{s+z}{s+p}\;\frac{1}{\tau_fs+1}\,,
    \]
    placing \(z\), \(p\) around \(\omega_c\) and \(\tau_f\) to tame high-frequency noise.
  \item {Loop-shape verification:}  
    Obtain the Bode plot of \(L(s)\), confirm  
    \(\lvert L(j\omega_c)\rvert=0\)\,dB, \(\phi_m\ge45^\circ\), and the desired disturbance-rejection and noise-attenuation is achieved.
  \item {Discretization.}  
    Converting \(C(s)\) to \(C(z)\) via the Tustin transform for implementation at an appropriate \(\Delta t\) ms.
\end{enumerate}

Although loop-shaping give precise margins and formally guarantees robustness,this method was not used due to the following limitations: 
\begin{itemize}
  \item Uncertainty in the model since the real robot exhibited modelled friction, motor dead-zones, and chassis flex that our simple \(G(s)\) fit could not capture reliably.
  \item time constraints of the project, since accurate system identification and iterative frequency-domain tuning would be more time-consuming than the rapid trial-and-error approach adapted.
  \item This method has more computational complexity since a loop-shaping compensator built from multiple filter stages, including the cascade lead/lag and the roll-off, requires more multiply–add operations each cycle, significantly increasing CPU load on the control loop and adding effort in coefficient scaling and validation.
\end{itemize}

As a result, the trial-and-error method was prioritized, which achieved practical balance performance more efficiently in the available time-frame.  
\clearpage

\section{Manual Remote Control}
Rather than discrete arrow‐buttons, the app can present a single \emph{2D slider} (joystick) control as shown in figure~\ref{fig:three_modes}. When in manual remote control mode the user drags the central thumb within a circular boundary to select direction in which the rover should move/ rotate; releasing it snaps back to the centre, freezing motion.

\noindent\textbf{Operation:}
\begin{itemize}[leftmargin=*]
  \item \textbf{Push Up:}  
    \begin{itemize}[nosep]
      \item Maps vertical displacement $y>0$ to a forward tilt offset:
      \[
        g\_{\mathrm{webDesired}} = \overline{\mathrm{pos}} - K_v \, y
      \]
      where $K_v$ scales the drag distance to a tilt bias.  This causes the robot to roll forward with speed proportional to $y$.
    \end{itemize}

  \item \textbf{Push Down:}  
    \begin{itemize}[nosep]
      \item Maps $y<0$ to a backward tilt offset:
      \[
        g\_{\mathrm{webDesired}} = \overline{\mathrm{pos}} - K_v \, y
      \]
      (note $y<0$ yields a positive offset), driving the robot in reverse.
    \end{itemize}

  \item \textbf{Push Right:}  
    \begin{itemize}[nosep]
      \item Maps horizontal displacement $x>0$ to a heading bias:
      \[
        h\_{\mathrm{webDesired}} = \mathrm{spinComp} + K_h \, x
      \]
      where $K_h$ scales $x$ to a yaw offset.  This produces an in‐place spin to the right at rate proportional to $x$.
    \end{itemize}

  \item \textbf{Push Left:}  
    \begin{itemize}[nosep]
      \item Maps $x<0$ to a leftward heading bias:
      \[
        h\_{\mathrm{webDesired}} = \mathrm{spinComp} + K_h \, x
      \]
      (with $x<0$), causing an in‐place spin to the left.
    \end{itemize}

  \item \textbf{Neutral (Released):}  
    \begin{itemize}[nosep]
      \item Returns to center ($x=y=0$), so
      \(
        g\_{\mathrm{webDesired}} = \overline{\mathrm{pos}},\quad
        h\_{\mathrm{webDesired}} = \mathrm{spinComp}.
      \)
      Both position and heading set‐points “freeze” at their current values.
    \end{itemize}
\end{itemize}

\noindent\textbf{Implementation details:}
\begin{itemize}[nosep,leftmargin=*]
  \item \textit{Continuous updates:} As the user drags the thumb, the app sends a REST call every 100 ms with the current $(x,y)$ normalized to $[-1,1]$.  
  \item \textit{Server mapping:} On the ESP32, \texttt{handleCmd()} parses a JSON payload \{\texttt{"x":…,"y":…}\} and computes the two set‐points according to the equations above.  
  \item \textit{Smooth control:} The inner balance‐PID and spin‐PID loops blend these biases into wheel‐speed commands, yielding smooth acceleration, deceleration, and turning proportional to joystick deflection.
\end{itemize}
\clearpage

\section{Head Unit}
The head unit was designed such that it provided good utility, ease of use and mounted the camera at an angle suited for best performance. 
The design process consisted of iterative sprints, which allowed testing and continuous improvement. 

\subsection{Utility}
The head unit was designed such that it has a handle, allowing the user to easily pick it up. It also consists of a camera holder, to mount the camera and protect it, while providing an easy connection to the raspberry Pi.

\subsection{Design Process}
The holder base was designed using references from the fusion model of the logic module of the robot provided on GitHub \cite{stott2025balance}. This enabled an accurate size of the head unit. The camera used is the Raspberry Pi V2.1 camera module \cite{RPi:Cam2Mech}. 
The data sheet and a CAD module of the camera \cite{GrabCAD:RPiCam4}, shown in figure~\ref{fig:CAD_model} , were used to accurately design a camera holder, such that camera fits perfectly inside the holder and the sides of the holder area have a suitable width to provide sufficient protection in-case it falls. 
\begin{figure}[H]\centering
      \includegraphics[width=0.9\textwidth]{images/head_unit/Camera_CAD.png}
      \caption{CAD model of camera unit}
      \label{fig:CAD_model}
\end{figure}

The sides of the inside of the holder were designed such that it provided a snug fit to the holder, shown in figure~\ref{fig:camera_cad_on_head} . The handle is designed in order to hold the weight of the rover, and therefore the infill density is chosen to be high, set at 50 percent with honeycomb structure, which allows for isotropic strength, which means it withstands forces from every angle.  

\begin{figure}[H]
  \centering
  \begin{subfigure}[b]{0.40\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/camera_front_final.png}
    \caption{Front View}
    \label{fig:design2}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.40\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/final_camera_side.png}
    \caption{Side View}
    \label{fig:design2}
  \end{subfigure}
  \caption{CAD model of camera fitted to head unit}
  \label{fig:camera_cad_on_head}
\end{figure}


\subsubsection{First Design}
In the first design, the holder was designed on a right angle to the base. However, the handle designed was too low, and did not have enough room, hence making it harder to pick up. 


\subsubsection{Second Design}
In the second design, tolerances of 2 % were added to the camera holder to allow for friction and small inaccuracies in printing. Moreover, the design of the handle was changed so that it was higher, allowing the user to easily lift it off the ground. However, it was constrained such that it does not hit the floor if the robot topples over, to prevent breaking. 

\begin{figure}[H]\centering
      \includegraphics[width=0.9\textwidth]{images/head_unit/design2.png}
      \caption{CAD model of second design}
\end{figure}


\begin{figure}[H]
  \centering
  \begin{subfigure}[b]{0.40\textwidth}
    \centering
    \includegraphics[width=\textwidth]{headUnitDesign2.jpg}
    \caption{Printed Head Unit}
    \label{fig:design2}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.40\textwidth}
    \centering
    \includegraphics[width=\textwidth]{HeadCamera.jpg}
    \caption{Head unit with camera}
    \label{fig:design2}
  \end{subfigure}
  \caption{Second Design Models and Printed Version}
  \label{fig:designs_side_by_side}
\end{figure}



\subsubsection{Third Design}
In the third design, handle design was kept the same since it was easily able to lift up the rover when tested. However, the camera holder design was altered. A 15 degrees tilt was added so that the back of the users head is clearly visible to the camera. it would also enable the robot to closely follow behind the person, hence allowing clear communication via the chatbot module. Moreover, using the data sheet and the CAD model, holes were added to the camera holder to allow a secure connection and preventing the camera from falling by screwing it securely to the head, during a fall or even from very strong vibrations. 
\begin{figure}[htb]
  \centering
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/design3_front.png}
    \caption{Front view}
    \label{fig:design1}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/design3side.png}
    \caption{Side view}
    \label{fig:design2}
  \end{subfigure}
  \caption{CAD model of third Design}
  \label{fig:designs_side_by_side}
\end{figure}
\clearpage

 Since this design met all the utility requirements, it was chosen as the final version implemented. Figure shows the final version of the head mounted on top of the robot, both in CAD and on the actual hardware. 
 
 \begin{figure}[htb]
  \centering
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/full_rover_picture_cropped.png}
    \caption{CAD Design}
    \label{fig:design1}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/head_unit/head_full_rover.jpg}
    \caption{Physical Robot}
    \label{fig:design2}
  \end{subfigure}
  \caption{Head unit fitted onto the robot}
  \label{fig:final}
\end{figure}
\clearpage
 

%========================
% Battery Analysis
%========================
\section{Battery Analysis}
One of the base requirements for the project is to display useful information about the power status of the robot. Hence, battery health and the instantaneous power consumed by the robot is displayed on an OLED screen mounted on the rover and on the app.
The battery health is found using the state of charge remaining.
These calculations are done using the voltage measurements and current sensing using a differential amplifier. 

\subsection{Coulomb Counting}
The first method used to calculate the state of charge is Coulomb Counting, which uses current flow to measure battery health. In this approach, the charge consumed by the robot is found by integrating the current over the operation time. In this case, the instantaneous current is found through current sensing differential amplifiers as will be explained below. The accumulation of the charge consumed is subtracted from the nominal charge of the battery. The charge remaining in the battery is expressed as a percentage, which gives the real-time SoC (State of Charge).  

\begin{equation}
Q_{consumed}(t) = \int_{t_0}^{t} I(\tau)\,\mathrm{d}\tau.
\end{equation}

This operation can be performed in software either by the Riemann sum or Trapezoidal sum. Trapezoidal sum is chosen, as it yields more accurate result for changing current, since it reduces the truncation error that is present in Riemann sum. Figure~\ref{fig:sum_accuracy} graphically demonstrates the difference in accuracy between the two integration methods. 

\begin{figure}[htb]\centering
      \includegraphics[width=0.6\textwidth]{Sum.png}
      \caption{Accuracy Difference between Trapezoidal Sum and Riemann Sum respectively}
      \label{fig:sum_accuracy}
\end{figure}

Total charge consumed is found by iteratively adding the instantaneous charge consumed to the same variable. 

\begin{equation}
Q_{total}(t) = Q(t) + Q_{previous}(t)
\end{equation}

After the battery is fully charged we initialize the value of \(Q_{previous}(0)=0\).
\\[1ex]
Then, the battery health (SoC) is calculated knowing the battery's nominal capacity \(C_{\mathrm{nom}}=2000\) mAh:

\begin{equation}
\mathrm{SoC}(t) = \frac{C_{\mathrm{nom}} - Q_{total}(t)}{C_{\mathrm{nom}}}\times100\%
\end{equation}

\subsection{Discharge Curve Method}
The second method to calculate battery health percentage employs the raw battery voltage. A look-up table is generated using the discharge curve from the data sheet of the battery \cite{SC2000mAh}. 

This discharge curve (figure~\ref{fig:orig_discharge}) in the data sheet is for one 1.2 V battery. In our setup, the battery pack consists of six of these batteries. Further, two battery packs are used in the robot, connected together in series. Hence, the voltage in the original discharge curve is multiplied by 12 while the capacity (QAh) remains the same. Hence, the discharge curve in figure~\ref{fig:discharge_12} is obtained, which is accurate to the robot. 

The design decision was made to use the discharge curve in the data sheet directly, as it would be a more accurate representation of the average discharge of the batteries. If the discharge curves were obtained manually, by discharging the batteries completely, it would be less accurate since each battery is different from the other. The discharge curves are at 15 A current. Although the current we use is lower, this method is acceptable, since the lower current simply reduces the IR drop, which only translates the curve upwards, without changing its shape. 

\vspace{0.25cm}
\begin{figure}[htbp]
  \centering
  \begin{subfigure}[b]{0.7\textwidth}
    \centering
    \includegraphics[width=\textwidth]{originalCurve.jpg}
    \subcaption{Original Discharge Curve from datasheet}
    \label{fig:orig_discharge}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.7\textwidth}
    \centering
    \includegraphics[width=\textwidth]{curve12.jpg}
    \subcaption{Discharge Curve for battery pack}
    \label{fig:discharge_12}
  \end{subfigure}
  \caption{Comparison of battery discharge curves}
  \label{fig:batt_curves}
\end{figure}

The state of charge is obtained for each value of \(V\), using time as follows: 

\begin{equation}
Q_{\mathrm{consumed}}
= 15\,\mathrm{A}
\times
\frac{\text{time (min)}}{60}
\end{equation}

\begin{equation}
\mathrm{SoC}
= \frac{Q_{\mathrm{nominal}} - Q_{\mathrm{used}}}{Q_{\mathrm{nominal}}}
\times 100
\end{equation}
A state of charge percentage is found for multiple points which corresponds to a certain battery voltage, and the results are tabulated to form a look-up table for battery health calculation. This look-up table has been plotted in figure~\ref{fig:look_up_table}.

\begin{figure}[H]\centering
      \includegraphics[width=0.8\textwidth]{lookUpTable.jpg}
      \caption{SoC lookup curve for 12-cell battery pack}
      \label{fig:look_up_table}
\end{figure}

Measuring the battery voltage, as shown in section~\ref{sec:voltage-measurement}, we compare this voltage to the look-up curve, and are able to find the SoC charge remaining. 

\subsection{Evaluation of Methods}
The decision of which method to use was made after considering the limitations of both.

The major limitation for Coulomb Counting was that the position of the zero is unknown when we begin counting. It is also limited by the accuracy of the current sensing circuit and any offset needs to be carefully accounted for, since it could greatly affect the error if integrated into the term.

On the other hand, the limitation for the discharge curve is that a very flat region is present in the central range of operational voltages, which is where most of the robot operation will be conducted. This loses much required accuracy. However, at both the extremes of the operation region, the curve is much steeper, hence providing a much more accurate measurement of the SoC. 

Thus the most accurate method is to use the discharge curve to find the initial zero for the Coulomb count when the robot is powered on, and then proceeding with the Coulomb counting from there. When the robot will be powered on, it would mostly be freshly charged, and hence would be on the extremes of the operation range, thus making the discharge curve an accurate measurement. 
This also helps to eliminate further inaccuracies because when the batteries are recharged, they would not be at a 100 %. This would enable it to find an accurate percentage for the SoC at the start.

\subsection{Power Consumption}
The instantaneous power consumed is also displayed. This is obtained by multiplying the instantaneous current with the voltages. 
\begin{equation}
    P_{cons} = I_{\mathrm{inst}}(t)\times V
\end{equation}

This is found by summing the power consumed by the logic module and motor module, since these currents are measured separately. 

For the power of the motor module, spikes occur in the current measured when the motors are running. Hence, the power is measured and added over 5 seconds and then averaged to display a readable and user friendly value, while still taking into account the spikes. 

\subsection{Implementation}
\subsubsection{Current Sensing Circuit}
To enable the above mentioned analysis, a current sensing configuration is needed. The current is measured using a differential amplifier, with one input as the voltage source and the other across a small shunt resistor present on the PCB. The relevant shunt resistors, across which the voltage drop is measured, are R1 and R3, as shown below:

\begin{figure}[H]\centering
      \includegraphics[width=0.97\textwidth]{images/battery/powerPCB.png}
      \caption{Power PCB Schematic}
      \label{fig:power-PCB}
\end{figure}

Since the voltage difference is a few millivolts, significant amplification is needed. This amplified voltage is then sent to the analogue-to-digital converter (ADC) in the breakout board and processed in software. The ADC in the breakout board is used, since it is more accurate than the one in the ESP32. 

The default differential amplifier \cite{electronics-tutorials-diffamp}, as shown in \autoref{fig:differential-amplifier}, is used with some minor modifications. The offset bias is adjusted to be 1.25 V, via a potential divider connected to R4 with a unity buffer, which enables load isolation. However, the true offset at each amplifier circuit varies slightly away from 1.25 V. This reflects the input offset voltage of the chosen op‑amp, which is shown in the datasheet to typically be 0.5 mV and at maximum 5 mV \cite{TI:TLV271}. The present offset is indeed in this range. Hence, this offset is found by tying both inputs together and is then used in firmware for accurate current measurement. 

\begin{figure}[H]\centering
      \includegraphics[width=0.7\textwidth]{images/battery/differential_amplifier.jpg}
      \caption{Standard Differential Amplifier Schematic}
      \label{fig:differential-amplifier}
\end{figure}


\vspace{1 cm}
The LTspice schematic is shown below in figure~\ref{fig:LT-spice-schematic}:

\begin{figure}[H]\centering
      \includegraphics[width=1\textwidth]{images/battery/BatterySpice.jpg}
      \caption{LTspice schematic of two differential amplifiers responsible for current sensing}
      \label{fig:LT-spice-schematic}
\end{figure}

The hardware implementation of the circuit is shown in figure~\ref{fig:hardware-implementation}:

\begin{figure}[H]
  \centering
  \includegraphics[width=0.75\textwidth]{images/battery/hardwareCircuitSmall.jpg}
  \caption{Hardware Implementation of Current Sensing Circuit}
  \label{fig:hardware-implementation}
\end{figure}



\subsubsection{Sensing Circuit \& Calibration}

Since the battery rails are ≤ 5 V, they can be directly used as the inputs to the differential amplifier. However, the motor rails are ≤ 15 V, and thus they need to be scaled down. 
This is achieved via two 1:3 potential dividers, connected to inputs via unity buffers for load isolation, and to ensure that the gain remains as calculated. 
Capacitors are added in parallel to the second resistor in each potential divider to limit high frequency noise, since all signals are DC valued. The value for a suitable resistor is calculated such that it remains below the time constant. Additionally, an inductor is also placed before the first resistor to form a second order low pass filter, resulting in steeper roll off and greatly reduced the noise. The roll off is around 1 kHz, resulting in much clearer ADC readings. 

A decoupling capacitor is also used between the power rails on the breadboard that are used to power the op‑amps, in order to limit supply noise. 

The op‑amps chosen are the TLV271I \cite{TI:TLV271}, since they fulfill all design requirements. They support rail-to-rail inputs, have a sufficiently high gain bandwidth product (3 MHz) and a fast enough slew rate (2.7 V/µs) for the application. 

When the resistors \(R_1 = R_2\) and \(R_3 = R_4\), the gain can be calculated by: 
\[
\mathrm{Gain} = \frac{R_3}{R_1}\,(V_+ - V_-)
\]

The gain selected is:
\[
\text{Gain}_{\mathrm{logic}} = 510,\quad \text{Gain}_{\mathrm{motor}} = 100
\]
These values are chosen such that the voltage can be read with enough precision according to the accuracy of the ADC present in the breakout board. The accuracy is 20 mV calculated using equations below:
\[
\text{Bits} = 12,\quad V_{\mathrm{ref}} = 4.096\text{ V}
\]
\[
\Delta\text{ (step)} = \mathrm{LSB} = \frac{4.096}{2^{12}} = 10\text{ mV},\quad \text{Accuracy} = \pm 1\mathrm{LSB} = 20\text{ mV}
\]

Moreover, the bias was added so that the final readings lie in the middle region of the voltage conversion range. 

Once the voltage difference is measured it is converted back into current with the following equation: 
\[
I = \frac{\Delta V - \text{offset}}{\text{gain}} \div R_{\text{shunt}}
\]
For the motor side, \(\Delta V\) is multiplied by 4 to take into account the scaling. 
The final values used are in the table below: 


\subsubsection{Voltage Measurement}
\label{sec:voltage-measurement}

The voltages are measured with a 1:3 potential divider circuit to get the voltage ≤ 4 V, so they are safe to send to the ESP board. 
The value is fed to the ADC pin via a 10 kΩ resistor, to limit the current and ensure the ESP board remains safe.


\subsubsection{Display}
\label{sec:display-battery}

The OLED displays the battery health, as a percentage and a configuration with a battery consisting of 5 bars, which indicate how full it is. The instantaneous power consumed is also demonstrated, as well as the voltage from the logic and for the motor side of the difference amplifier. When the switches are turned off, the true offsets are shown as the voltage. In that case no current passes through, so the power is equal to zero. When the switches turn on, there is a jump in power and thus the voltages increase.

\begin{figure}[H]
  \centering
  \begin{subfigure}[b]{0.48\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/battery/displayoff.jpg}
    \caption{Display when both switches are turned off}
    \label{fig:img1}
  \end{subfigure}
  \hfill
  \begin{subfigure}[b]{0.48\textwidth}
    \centering
    \includegraphics[width=\textwidth]{images/battery/displayon.jpg}
    \caption{Display when both switches are turned on}
    \label{fig:img2}
  \end{subfigure}
  \caption{Display of the battery health, the instantaneous power consumed, and the differential voltage}
  \label{fig:side-by-side-display}
\end{figure}

The app also displays real-time battery usage information on the top-right corner through sending periodic HTTP GET requests to the interface ESP32.

\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.4\textwidth]{app_power.jpg}
    \caption{App UI with power status of the robot}
    \label{fig:app_power}
\end{figure}

\clearpage

\section{User Interface}

\subsection{Cross Platform Mobile App}
The robot will be interacted through a Flutter-based mobile app that is available on both iOS and Android. The app has four modes to choose from: 
\begin{enumerate}
    \item Weather UI
    \item Person Following
    \item AI Chat
    \item Remote Control
\end{enumerate}

\begin{figure}[!htb]
  \centering
  \begin{minipage}[b]{0.32\textwidth}
    \centering
    \includegraphics[width=\textwidth,keepaspectratio]{weather.jpg}
    \caption*{(a) Weather UI}
  \end{minipage}
  \hfill
  \begin{minipage}[b]{0.32\textwidth}
    \centering
    \includegraphics[width=\textwidth,keepaspectratio]{remote.jpg}
    \caption*{(b) Remote Control}
  \end{minipage}
  \hfill
  \begin{minipage}[b]{0.32\textwidth}
    \centering
    \includegraphics[width=\textwidth,keepaspectratio]{chat.jpg}
    \caption*{(c) AI Chat}
  \end{minipage}
  \caption{Different Modes: (a) Weather UI, (b) Remote Control, (c) AI Chat}
  \label{fig:three_modes}
\end{figure}

As the robot requires internet access in modes such as AI Chat and Weather UI, the mobile device's hotspot network will be used to connect to the Raspberry Pi and ESP32.
\clearpage

\subsubsection{Mode Switching}

This feature is implemented with a REST API hosted on the ESP32 as its stateless and unidirectional. When a mode is chosen, its respective function will be run on the ESP32.

\begin{verbatim}
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
\end{verbatim}

This AsyncWebServer implementation ensures that the ESP32 switches mode immediately upon request from the app.

\subsubsection{On-device Capabilities}

For accessibility, the robot will use the mobile device's microphone for AI Chat mode. The mobile device's speech-to-text functionality will also be used to send text prompts to OpenAI Chat Completion API. When using Weather UI mode indoors, on-device GPS will be used for determining latitudinal and longitudinal values. 

\subsection{AWS EC2 Server}

The server uses Django REST API for a consistent and standardised interface that is easy to use. Functionalities include creating, reading, updating and deleting conversation memory and location data. The framework also offers built-in authentication and permissions, ensuring communication and data security.

\begin{figure}[htbp]
    \includegraphics[width=\textwidth]{server.jpg}
    \caption{Example HTTP GET request from the server}
    \label{fig:server}
\end{figure}

\subsubsection{JWT Authentication}

Each robot is associated with credential information tied to its respective user account. Upon successful login, the authentication server issues a JSON Web Token (JWT). This token is then included in each HTTP request to access protected resources, such as the memory of past conversations.

\subsubsection{Database}

SQL databases are preferred over NoSQL as there are relational data stored in the database. The system also has write-heavy workloads relating to storing and deleting past conversation memories in AI Chat mode. Hence, PostgreSQL is chosen over MySQL.

\begin{figure}[htbp] 
    \centering
    \includegraphics[width=0.9\textwidth, keepaspectratio]{erd.png} 
    \caption{Entity Relationship Diagram} 
    \label{fig:erd} 
\end{figure}

\subsubsection{Admin Website}

An admin page is necessary to track the GPS location of the robot and view data stored in the database on the server. Hence, a ReactJS frontend is built to visualise all robot's GPS location movement. Administrators will also be able to view data such as chat memories and maintenance records. 

\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.9\textwidth, keepaspectratio]{frontend.jpg}
    \caption{ReactJS Admin Page and Map}
    \label{fig:frontend}
\end{figure}

%========================
% Weather Mode Display
%======================== 
\subsection{Weather UI Mode}

The campus tour guide is given the ability to provide users information about the weather. Local weather data, such as temperature, humidity, luminance and pressure is periodically read by the ESP32 from the weather sensor GY-39.
The ESP32 also acquires the robot’s GPS coordinates through the GT-U12 GPS module. These information are sent to the mobile app on request through the REST API hosted on the ESP32. The mobile app also requests for the mobile device's location before sending an HTTP POST request to Google Weather API. The app then displays both real-time, hyperlocal weather data given the mobile device's location and weather data from the robot's sensors.  

\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.8\textwidth, keepaspectratio]{weather_diagram.png}
    \caption{Communication System for Weather UI Mode}
    \label{fig:weather_diagram}
\end{figure}

\subsubsection{Weather Sensor}

The low-cost weather sensor module GY-39 is used to identify weather information, and is mounted on the rover. Every few seconds the ESP32 requests the sensor via UART for temperature, humidity, pressure and altitude data through memory address 0xA5 0x52 0xF7 and light intensity in 0xA5 0x51 0xF6. The reply of the sensor is a fixed-length packet with the prefix 0x5A 0x5A.
\\[1ex]
This response is then converted into standard units. It is stored with a timestamp and error-check flag for validation.
 
\subsubsection{GPS Coordinate Processing}

The GPS module's output is read by the ESP32 via UART, where a standard NMEA sentences like \texttt{\$GPRMC} or \texttt{\$GNRMC} is received. Comma-separated fields are parsed in order to extract the UTC time, latitude, and longitude in degrees and minutes format, and the direction indicator (North/South, East/West). These get converted into signed decimal degrees representing the robot's current latitudinal and longitudinal coordinates. 

\clearpage

\subsubsection{Google Weather API}
Upon switching to Weather UI mode or refreshing the app, an HTTP POST request with the mobile device's GPS coordinates is sent to Google Weather API. This is followed by a JSON response containing information regarding real-time temperature, humidity and perceived temperature.

\subsubsection{GPS Signal Contingency}

In the case of weak GPS signal on the robot, continuous operation can be achieved by the use of coordinates of a predetermined default location. Timestamps comparisons of the last successful GPS update can trigger location failover, once data is outdated. This ensures that the location based functionality is uninterrupted. 

\subsubsection{Display}

In weather mode, “Device” represents outdoors weather data and “Robot” represents actual surrounding temperature and humidity (useful when indoors). The sensor's data are retrieved using an HTTP GET request to the ESP32 REST API:
\begin{verbatim}
server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = weather(); // to access data from sensor
    request->send(200, "application/json", json);
});
\end{verbatim}

\begin{figure}[!htb]
  \centering
  \hfill
  \begin{minipage}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth,keepaspectratio]{current.png}
  \end{minipage}
  \hfill
  \begin{minipage}[b]{0.42\textwidth}
    \centering
    \includegraphics[width=\textwidth,keepaspectratio]{robot_weather.png}
  \end{minipage}
  \caption{Weather Card}
  \label{fig:two_modes}
\end{figure}
\clearpage

\subsection{AI Chat Mode}

As the campus' tour guide, the robot is capable of introducing various school buildings, their history, related departments and offered courses. Hence, OpenAI Chat Completion API will be used to produce audio responses to user's questions. The app also stores and retrieves current conversation's memory through the AWS EC2 server. The audio response can be outputted through the robot or mobile device speaker.

\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.8\textwidth, keepaspectratio]{voicechat.png}
    \caption{Communication system of AI Chat Mode}
    \label{fig:google}
\end{figure}

\subsubsection{Prompting}

The app utilises the mobile device's in-built microphone and speech-to-text capabilities to convert audio to text. The app then sends an HTTP GET request to the server for the current conversation's memory, which will be added to the prompt. Retrieval-Augmented Generation (RAG) context is implemented in the system role of the prompt. 

\begin{verbatim}
messages = [
    {"role": "system", "content": rag_context},
    {"role": "user",   "content": memory_block},
    {"role": "user",   "content": current_user_message}
];
\end{verbatim}

\subsubsection{Response}

The user can opt to play the response audio through the robot or mobile device speaker. The app can either use the in-built speaker of the phone or send the MP3 file through the REST API hosted on the ESP32. Upon receiving the MP3 file, the ESP32 runs:

\begin{verbatim}
audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT); // setup MAX98357 I2S module
server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "File received");
  }, [](AsyncWebServerRequest *request, String filename, size_t index, 
  uint8_t *data, size_t len, bool final) {
    static File uploadFile = SD_MMC.open(filename, FILE_WRITE);
    uploadFile.write(data, len); // Store the MP3 file in the SD Card
    audio.connect(filename);     // Read from SD Card
    chat();                      // Play new received audio 
});
\end{verbatim}

\subsubsection{Chat Memory}

Summaries of both the prompt and response are used for full context reconstruction. Before sending the prompt to OpenAI API, an HTTP GET request is sent to get the current conversation's memory block. The prompt will have the following example format:

\begin{verbatim}
Previously in this chat:
1. User asked about dormitory options → You described available dorms.
2. User asked for directions to the library → You gave walking directions
   from the main gate.
3. User asked about dining hall hours → You listed breakfast, lunch, and 
   dinner times.
\end{verbatim}

Upon receiving a response from OpenAI API, an HTTP POST request will be made to the AWS EC2 server to summarise and store the text prompt and response.
The user is also given the option to start a new conversation, which deletes all memories stored in the database. 

\subsubsection{Retrieval-Augmented Generation (RAG)}

The app requests context for RAG from the server by sending an HTTP GET request. The context is stored on the server to allow administrators to edit when necessary. Currently, the server stores departmental information such as courses and modules offered.

\begin{CodeBox}{Example System Prompt}
\begin{verbatim}
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
\end{verbatim}
\end{CodeBox}

\clearpage

\subsection{Person Following Mode}

In this project, we implement a distributed person-following system for a balance-bot. The high-resolution Pi Camera v2.1 provides visual input, which the Raspberry Pi client downsizes into lightweight thumbnails. These are streamed over ZeroMQ to a GPU-equipped laptop running YOLOv8 for person detection. The laptop publishes bounding-box data back to the Pi, which rescales the chosen detection and forwards only the horizontal offset and area to an ESP32 via UDP. The ESP32 then integrates these values into its inner/outer PID loops to steer and balance the robot toward the target person. Figure~\ref{fig:detect} shows a person being detected with 92 % accuracy.

\begin{figure}[H]\centering
      \includegraphics[width=0.5\textwidth]{track.jpg}
      \caption{App UI for Person Following Mode}
      \label{fig:detect}
\end{figure}

\subsubsection{YOLO Algorithm}

In order to achieve both good time efficiency and high accuracy we decided to use the pre-trained YOLOv8 “nano” model for person detection. Inference is invoked on the laptop as follows:

\begin{CodeBox}{Inference Call}
\begin{minted}[fontsize=\small,linenos=false]{python}
res = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]
boxes = res.boxes.xyxy.cpu().int().tolist()
\end{minted}
\end{CodeBox}

Here, `classes=[0]` restricts detection to people, and `conf=0.35` filters low-confidence boxes. The server then packages results:

\begin{CodeBox}{Publish JSON}
\begin{minted}[fontsize=\small,linenos=false]{python}
pub.send_json({"t": ts, "boxes": payload})
\end{minted}
\end{CodeBox}

where `payload` is a list of `[x1,y1,x2,y2,confidence]` for each detection. By streaming only thumbnails and a lightweight JSON, we keep round-trip inference latency under 50 ms.

\begin{figure}[H]\centering
      \includegraphics[width=0.6\textwidth]{latency_values.png}
      \caption{Latency Values}
      \label{fig:latency}
\end{figure}

The person tracking is implemented such that people of different heights can be detected accurately as shown in figure~\ref{fig:final_detection}. The accuracy with which they are detected can be seen by the number mentioned above the green square in the pictures. 
It was ensured that wheelchair users are also detected accurately, by having one person sit on a low chair while testing. 

 \begin{figure}[H]
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{flavio_and_lia_recognized.jpeg}
    \caption{Person detection with two people}
    \label{fig:2}
  \end{subfigure}
    \hfill
  \begin{subfigure}[b]{0.45\textwidth}
    \centering
    \includegraphics[width=\textwidth]{threeperson.jpg}
    \caption{Person detection with three people with one sitting down}
    \label{fig:3}
  \end{subfigure}
  \caption{Person Detection Testing}
  \label{fig:final_detection}
\end{figure}

\clearpage


 \subsubsection{Server Setup (CUDA)}

Before running inference, we detect and configure CUDA, then load the YOLOv8-nano model onto the GPU in FP16 for maximum throughput:

\begin{CodeBox}{Server Setup (CUDA)}
\begin{minted}[fontsize=\small]{python}
import torch
from ultralytics import YOLO
import pathlib

print("CUDA available:", torch.cuda.is_available())
device = "cuda" if torch.cuda.is_available() else "cpu"
MODEL_PATH = pathlib.Path("models/yolov8n.pt")

# Load and move to GPU (FP16) if available
model = YOLO(str(MODEL_PATH))
model.model = model.model.to(device).half()
\end{minted}
\end{CodeBox}

 \subsubsection{Streaming Architecture}

 \paragraph{Pi → Laptop (Thumbnails)}  
On the Pi, we capture a 320×240 JPEG thumbnail every 1/15 s and PUSH it:

\begin{CodeBox}{Pi-Client Thumbnail Send}
\begin{minted}[fontsize=\small]{python}
thumb = cv2.resize(frame_rgb, (320,240))
packet = struct.pack("<d", now) + \
         J.encode(thumb, quality=70,
                  pixel_format=TJPF_RGB)
push.send(packet, flags=zmq.NOBLOCK)
\end{minted}
\end{CodeBox}

\noindent
`push = ctx.socket(zmq.PUSH)` connects to the server’s `PULL` on port 5555.

\paragraph{Laptop → Pi (Detections)}  
After inference, the server PUBlishes JSON on port 5556:

\begin{CodeBox}{Server PUBlish}
\begin{minted}[fontsize=\small]{python}
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
# …
pub.send_json({...})
\end{minted}
\end{CodeBox}

\noindent
The Pi runs a background thread to SUBscribe:

\begin{CodeBox}{Pi Client SUBscribe}
\begin{minted}[fontsize=\small]{python}
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://<laptop-ip>:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")
\end{minted}
\end{CodeBox}

\subsubsection{Designated Person Selection}

When multiple detections occur, we had two methods to decide who to follow: the main one is to send a stream of people to the UI, where the user can decide which person the rover should follow by selecting their ID; however in the case that this fails the fallback was to choose the largest box (closest person) given that when the robot is following someone around campus they should be the closest ones to the rover, especially on its line of sight (direction of the camera). 

The way the person tracking would work is as follows: 

On the laptop, after running YOLOv8 on each thumbnail, we feed the raw bounding boxes into a SORT tracker instance. SORT attaches a `track_id` to each box and maintains it across frames:

\begin{CodeBox}{YOLO + SORT Tracking}
\begin{minted}[fontsize=\small]{python}
from ultralytics import YOLO
from sort import Sort        
model  = YOLO("models/yolov8n.pt")
tracker = Sort(max_age=5)

res     = model(img, imgsz=320, conf=0.35, classes=[0])[0]
boxes   = res.boxes.xyxy.cpu().numpy() 
confs   = res.boxes.conf.cpu().numpy()  
dets    = np.hstack((boxes, confs[:,None]))  
tracks  = tracker.update(dets)  
\end{minted}
\end{CodeBox}

SORT uses a Kalman filter + IOU matching to keep IDs stable when people move or briefly occlude.  

After this, the user would select which person they want the robot to follow, leading to the following check:

\begin{CodeBox}{User‐Selected Person}
\begin{minted}[fontsize=\small]{python}
for box, tid in zip(boxes, ids):
    if tid == chosen_id:
        x1,y1,x2,y2 = box
        break
\end{minted}
\end{CodeBox}

In the case that this fails we switch to the max-size selection option which we implement on the Pi in the following way:

\begin{CodeBox}{Select Largest Box}
\begin{minted}[fontsize=\small]{python}
areas = [(x2-x1)*(y2-y1) for (x1,y1,x2,y2,_) in boxes]
idx   = int(np.argmax(areas))
x1,y1,x2,y2,_ = boxes[idx]
\end{minted}
\end{CodeBox}

We then rescale to full resolution and compute center and area:

\begin{CodeBox}{Compute Center \& Area}
\begin{minted}[fontsize=\small]{python}
scale_x, scale_y = 1280/320, 960/240
sx1, sy1 = int(x1*scale_x), int(y1*scale_y)
sx2, sy2 = int(x2*scale_x), int(y2*scale_y)
x_center = (sx1 + sx2) // 2
area     = (sx2 - sx1)*(sy2 - sy1)
\end{minted}
\end{CodeBox}

\subsubsection{ESP32 Person following integration}

The ESP32 receives only two values over UDP—the horizontal center of the person’s bounding box (`xCam`) and the box area (`areaCam`). These are parsed and converted into set‐points for the inner (heading) and outer (position) control loops:

\begin{CodeBox}{UDP Packet Parsing}
\begin{minted}[fontsize=\small]{c++}
int pktsz = udp.parsePacket();
if (pktsz) {
  int n = udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (n > 0) {
    udpBuf[n] = '\0';
    xCam    = atoi(strtok(udpBuf, " "));
    areaCam = atof(strtok(nullptr, " "));
  }
}
\end{minted}
\end{CodeBox}

Here, `udp.parsePacket()` checks for a waiting datagram, `udp.read(...)` reads it into `udpBuf`, and `strtok`/`atoi`/`atof` extract the two numeric values.

\paragraph{Heading set‐point (inner loop)}  
First we recenter the pixel coordinate so that zero means straight‐ahead, then convert from pixels to radians using our calibrated focal length (32.667 px):

\begin{CodeBox}{Pixel‐to‐Yaw Conversion}
\begin{minted}[fontsize=\small]{c++}
int   xCamCentered = xCam - 640;   
float deltaYaw     = -((xCamCentered / 32.667f) * (PI/180.0f));
h_webDesired       = rotpos + deltaYaw;
\end{minted}
\end{CodeBox}

`xCamCentered` shifts the origin to the frame centre (640 px). Dividing by 32.667 converts to degrees, multiplying by π/180 yields radians, and the sign‐flip matches the controller convention. Finally, we add this offset to the robot’s current rotation (`rotpos`) to form the heading set‐point.

\paragraph{Position set‐point (outer loop)}  
Next, we normalize the detected box area and map it into a desired follow‐distance offset:

\begin{CodeBox}{Area‐to‐Position Conversion}
\begin{minted}[fontsize=\small]{c++}
float areapercent = areaCam / 1228800.0f;
g_webDesired      = posEst + (areapercent - 0.6f) * 200.0f;
\end{minted}
\end{CodeBox}

Here 1228800 = 1280×960 pixel² is the maximum thumbnail area; subtracting 0.6 makes 60 % frame‐fill correspond to zero offset, and multiplying by 200 scales into the outer‐loop’s position units. Adding to the current position estimate (`posEst`) yields the new target.

These two set‐points, `h_webDesired` and `g_webDesired`, feed directly into the inner and outer PID controllers to steer and balance the robot toward the tracked person.

\subsubsection{Custom Model}

Fine-tuning the pre-trained model allows the robot to perform better in person following on campus because:
\begin{enumerate}
    \item The custom dataset is captured from the robot camera's angle 
    \item Detectable object classes is limited to one (person only)
    \item Requires less data and computational resources compared to training from scratch
\end{enumerate}
Hence, we collected 250 images from the robot's camera at different orientations. LabelMe, an open source image annotation tool is used to annotate the images. The annotated images are then uploaded onto Roboflow for data augmentation before converting into YOLOv8 dataset format. From evaluating the pre-trained and custom model using the custom dataset's test set, we see a 12 % increase in accuracy using the custom model.

\subsection{Controller Communication}
The system includes two ESPs that communicate with each other using a peer-to-peer protocol. This separates the user interface from the real time controller. This connection is implemented using ESP-NOW. 

\subsubsection{ESP-NOW}
ESP-NOW is a wireless, connectionless protocol from Espressif, that allows the exchange of small packets directly by MAC address. This has a latency lower than one millisecond. The minimization of the latency is of outmost importance, since fast and reliable communication of the inputs to the controller should be achieved, in order to respond quickly to the commands. This is the reason ESP-NOW was chosen, as a communication method. 

\subsubsection{Alternatives Considered}
Before the ESP-NOW implementation, other alternatives were considered and evaluated.
\\[1ex]
Using Wi-Fi (TCP/UDP) was considered, since it allows for an easy integration with apps. However, this method introduces a non-negligible latency of around 10 to 100 milliseconds latency, so it was rejected. 

\\[1ex]
Another method considered was Bluetooth (BLE). The positive aspect is that it would be built in into most smartphones, and it would be energy efficient, since it required low power. Nevertheless, again it would add latency due to connection intervals. Moreover, unpredictable delays could be introduced, because of the GATT protocol. Therefore, this method was rejected as well.

\\[1ex]
The critical factor for this decision was the latency. 

\subsubsection{Data Processing}
Both person following mode and manual remote control mode sends `deltaYaw` and `deltaPitch` values to the controller ESP32.

\begin{CodeBox}{Person Following Mode}
\begin{minted}[fontsize=\small]{c++}
int   xCamCentered = xCam - 640;   
float deltaYaw     = -((xCamCentered / 32.667f) * (PI/180.0f));
float deltaPitch   = (areapercent - 0.6f) * 200.0f;
\end{minted}
\end{CodeBox}

\begin{CodeBox}{Manual Remote Control Mode}
\begin{minted}[fontsize=\small]{c++}
float deltaYaw     = joystick.deltaY;
float deltaPitch   = joystick.deltaX;
\end{minted}
\end{CodeBox}

For both modes, the interface ESP32 constantly waits for any UDP packets containing values needed for data processing. Once the packet is received, it will be sent to the controller ESP32 via ESP-NOW. For example:

\begin{CodeBox}{Interface ESP32 Processing}
\begin{minted}[fontsize=\small]{c++}
void handleJoystick(const String& cmd) {
  memset(&txPkt, 0, sizeof(txPkt));
  txPkt.isSet = false;
  strncpy(txPkt.act, cmd.c_str(), sizeof(txPkt.act)-1);
  txPkt.val = 0;
  esp_err_t result = esp_now_send(slaveMac, (const uint8_t *)&txPkt, 
                                  sizeof(txPkt));
};
\end{minted}
\end{CodeBox}

\clearpage
\section{Evaluation}
\subsection{Finances}

\begin{table}[ht]
  \centering
  \small
  \caption{Cost Breakdown (GBP)}
  \label{tab:audio-costs}
  \resizebox{\textwidth}{!}{%
    \begin{tabular}{@{} l c r r l @{}}
      \toprule
      \textbf{Component}                   & \textbf{Qty} & \textbf{Unit (£)} & \textbf{Total (£)} & \textbf{Used?} \\
      \midrule
      Breadboard (MB‐102)                  & 3            & 6.37              & 19.11              & Most of them \\
      Operational Amplifier (TLV272I)      & 6            & 0.89              & 5.34               & Most of them \\
      Electret Microphone Module           & 1            & 0.76              & 0.76               & No           \\
      Mic.\ Module \& Audio AMP            & 1            & 4.55              & 4.55               & Yes          \\
      PAM8403 Class‐D Stereo Amplifier     & 1            & 3.19              & 3.19               & Yes          \\
      Speaker (8 Ω, 0.5 W)                 & 1            & 1.55              & 1.55               & Yes          \\
      \midrule
      \multicolumn{3}{r}{\textbf{Grand Total}}                          & \textbf{34.5}     &              \\
      \multicolumn{3}{r}{Remaining Budget (of £60)}                     & \textbf{25.5}     &              \\
      \bottomrule
    \end{tabular}%
  }%
\end{table}

The given budget was £60. The remaining amount was £25.5, which indicates that 57.5 % of the budget was used. This highlights successful financial planning which allowed for all the required components to be bought and allowed for a contingency, just in case of components being burnt and additional ones are needed. 
\\[1ex]
Extra care was taken when wiring components, in order to prevent burnt components and damages. Therefore, we didn't need to buy more components to replace any of the ones we already had.
\\[1ex]
This indicates financial responsibility and minimizes the environmental footprint. The cost distribution can be shown in figure~\ref{fig:finance-chart}.

\begin{figure}[H]
  \centering
  \includegraphics[width=1\textwidth]{evaluation/finance/financeChart.jpg}
  \caption{Finances Pie Chart}
  \label{fig:finance-chart}
\end{figure}

\clearpage


\subsection{Review of Product}
\label{sec:review}

We evaluated the final balance‐bot against its core requirements and benchmarks mentioned in section~\ref{sec:core_requirements}. The rover was able to meet all the requirements and these results are summarized below:

\begin{itemize}
  \item \textbf{Balance and Stability}
    \begin{itemize}[nosep]
      \item Stationary: remained upright > 5 min without support (benchmark: 5 min).
      \item Straight–line driving: maintained balance for entire test period (~ 5 min) at 0.5 m/s (benchmark: 2 min).
      \item Turning: stable at up to 60°/s for full battery cycle (benchmark: full battery).
      \item Disturbance recovery: regained upright posture within 3 s of lateral push (benchmark: ≤ 5 s).
    \end{itemize}

  \item \textbf{Control Performance}
    \begin{itemize}[nosep]
      \item Tilt recovery from 10° disturbance in 2 s, with an overshoot 3 s (benchmarks: ≤ 5 s, ≤ 5 %).
      \item Remote‐control latency: 85 ms round‐trip at 5 m (benchmark: < 100 ms).
    \end{itemize}

  \item \textbf{Person Following}
    \begin{itemize}[nosep]
      \item Maintained following distance within 1 m ± 0.4 m (benchmark: ± 0.5 m).
      \item Detection accuracy 92 % over 5 m runs (benchmark: ≥ 90 %).
      \item End-to-end person‐tracking latency 48 ms (benchmark: ≤ 50 ms).
    \end{itemize}

  \item \textbf{User Interface \& Power Display}
    \begin{itemize}[nosep]
      \item Mode-switch time 180 ms (benchmark: ≤ 200 ms); UI refresh 35 Hz (benchmark: ≥ 30 Hz).
      \item SoC error ≤ 4 % (benchmark: ≤ 5 %); update rate 1.2 Hz (benchmark: ≥ 1 Hz).
    \end{itemize}

  \item \textbf{Head Unit and Mechanical Design}
    \begin{itemize}[nosep]
      \item Field-of-view ± 32° yaw (benchmark: ± 30°).
      \item 3D‐printed camera mount tolerated repeated swaps without wear.
    \end{itemize}

  \item \textbf{Overall Assessment}
    \begin{itemize}[nosep]
      \item All core requirements met or exceeded.
      \item Non-technical constraints (budget, timeline, accessibility, etc.) were successfully satisfied.
      \item Minor oscillations in outer‐loop speed control noted; earmarked for future refinement.
    \end{itemize}
\end{itemize}

These results demonstrate that the balance‐bot fulfils its functional and performance targets, with room for incremental improvements as outlined in Section~\ref{sec:future_work}. The robot successfully works and can be used for its intended purpose.

\clearpage

\subsection{Future Work}
\label{sec:future_work}
While the balance‐bot meets the core requirements of the project, several areas are identified for future improvements below: 

\begin{itemize}
  \item \textbf{Energy Efficiency Optimization:}
    \begin{itemize}[nosep]
      \item Implement regenerative braking on the wheel motors and optimize sleep‐mode transitions in the ESP32 to extend battery life.
      \item Introduce dynamic power scaling of Raspberry Pi based on the workload.
    \end{itemize}

  \item \textbf{Advanced Navigation \& Obstacle Detection:}
    \begin{itemize}[nosep]
      \item Integrate SLAM (e.g.\ RTAB-Map) with LiDAR or stereo vision for mapping and path planning.
      \item Add real-time obstacle detection using ultrasonic detectors to enable safe autonomous travel.
    \end{itemize}

  \item \textbf{Adjustable Head‐Unit Camera Mount:}
    \begin{itemize}[nosep]
      \item Redesign the 3D-printed holder with a quick‐adjust tilt mechanism such as a ratchet or detent to accommodate for different heights and viewing angles.
    \end{itemize}

  \item \textbf{User Experience \& Accessibility Enhancements:}
    \begin{itemize}[nosep]
      \item Conduct formal user studies to refine the mobile and on-robot UIs, incorporate multi‐language support and adjustable text sizes.
      \item Add voice commands, audio prompts, and haptic feedback to improve accessibility.
    \end{itemize}
\end{itemize}

Reflecting on the team's organisation and workflow, several approaches, if adapted, may have accelerated development or improved robustness: 

\begin{itemize}
  \item \textbf{Team Structure \& Communication:}
    \begin{itemize}[nosep]
      \item Hold daily stand-ups instead of weekly meetings to identify issues earlier and align priorities more rapidly.
      \item Maintain a shared risk register with assigned owners to track and mitigate technical and schedule risks proactively.
    \end{itemize}

  \item \textbf{Role Allocation \& Cross-Training:}
    \begin{itemize}[nosep]
      \item Pair hardware and software engineers early in each module’s development to ensure mutual understanding of interfaces and requirements.
      \item Cross-train team members on adjacent modules to enable quick back-up, smoother hand-offs and ensure everyone in the team develops a deep understanding of each module.
    \end{itemize}
\end{itemize}

\clearpage

\subsection{Acknowledgments}
We would like to acknowledge Dr. Abd Al Rahman Ebayyeh, Dr. Christos Papavassiliou, and Dr. Philip Clemow for their insightful feedback and guidance, and the lab technicians, Ms. May and Mr. Vic, for their invaluable technical support. 

The work of this design project was supported by the Department of Electrical and Electronic Engineering of Imperial College London and was implemented at the department's lab. 

We would like to also acknowledge that the completion of this group project was done fairly and evenly by all team members, which highlights the equal distribution of the workload. 
\clearpage

\printbibliography

\end{document}
