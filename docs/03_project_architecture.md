\# ARGOS Project Architecture



\*\*Document ID:\*\* DOC-003

\*\*Document Title:\*\* Project Architecture

\*\*Version:\*\* 1.0 (Draft)

\*\*Status:\*\* Active

\*\*Project:\*\* ARGOS – Autonomous Robotic Guard \& Observation System



\---



\## 1. Purpose



This document defines the overall architecture of the ARGOS platform.



Rather than describing implementation details, this document explains how the major subsystems interact, what responsibilities they have, and how the platform is expected to evolve throughout future development phases.



The architecture described here serves as the foundation for all future hardware, firmware and software development.



\---



\## 2. Architectural Philosophy



ARGOS follows a layered and modular architecture.



Each subsystem has a clearly defined responsibility and communicates with other subsystems through well-defined interfaces.



Subsystems should remain as independent as practical in order to simplify testing, debugging and future upgrades.



Whenever possible, functionality should be separated rather than combined.



\---



\## 3. High-Level Architecture



The ARGOS platform is divided into seven major subsystems.



```text

\\\&#x20;                   ARGOS



\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │      Mission Layer (Future)      │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │   Communication Layer (Future)   │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │     Decision Layer (Future)      │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │     Perception Layer             │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │    Motion Control Layer          │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │      Hardware Layer              │

\\\&#x20;     └──────────────────────────────────┘

\\\&#x20;                   │

\\\&#x20;     ┌──────────────────────────────────┐

\\\&#x20;     │        Power Layer               │

\\\&#x20;     └──────────────────────────────────┘

```



Every layer depends only on the services provided by the layer directly beneath it.



This approach minimizes coupling and improves maintainability.



\---



\## 4. Power Layer



The Power Layer is responsible for supplying stable electrical power to every subsystem.



Responsibilities include:



\- Battery distribution

\- Voltage regulation

\- Power monitoring

\- Battery protection

\- Emergency shutdown support



Future revisions will include:



\- Charging dock support

\- Battery health estimation

\- Intelligent charging



\---



\## 5. Hardware Layer



The Hardware Layer represents all physical electronics responsible for interacting with the real world.



Subsystems include:



\- STM32 Controller

\- Motor Driver

\- Sensor Interfaces

\- Encoder Inputs

\- Debug Interfaces



The hardware layer provides deterministic real-time operation.



\---



\## 6. Motion Control Layer



The Motion Control Layer transforms motion commands into physical wheel movement.



Responsibilities include:



\- PWM generation

\- Motor direction control

\- Encoder processing

\- RPM calculation

\- PID speed control

\- Differential drive



Future support:



\- Motion profiling

\- Trajectory execution

\- Self-balancing controller



\---



\## 7. Perception Layer



The Perception Layer allows ARGOS to understand its surroundings.



Phase 1 sensors:



\- Wheel Encoders

\- IMU

\- ToF Sensors



Future sensors:



\- Camera

\- LiDAR

\- Docking Sensors

\- Environmental Sensors



Responsibilities:



\- Sensor acquisition

\- Data filtering

\- Environment estimation



\---



\## 8. Decision Layer



The Decision Layer determines what the robot should do.



Initially this layer is intentionally simple.



Responsibilities:



\- State machine

\- Safety monitoring

\- Motion requests



Future responsibilities:



\- Navigation

\- Mission planning

\- Obstacle avoidance

\- Autonomous behaviors



The Decision Layer should never directly control hardware.



Instead, it communicates with the Motion Control Layer.



\---



\## 9. Communication Layer



The Communication Layer exchanges information with external systems.



Phase 1:



\- UART

\- USB Debug



Future:



\- Wi-Fi

\- MQTT

\- Home Server

\- Remote Dashboard



Responsibilities:



\- Telemetry

\- Diagnostics

\- Remote commands

\- Logging



The communication subsystem shall remain optional.



The robot must always remain operational without network connectivity.



\---



\## 10. Mission Layer



The Mission Layer represents the highest level of system intelligence.



It is responsible for defining why the robot is moving rather than how it moves.



Examples include:



\- Patrol

\- Return Home

\- Standby

\- Investigation

\- Manual Control

\- Docking



Future AI-assisted behaviors may also be introduced within this layer.



\---



\## 11. Data Flow



A typical execution sequence is shown below.



```text

Mission

\\\&#x20;  ↓

Decision

\\\&#x20;  ↓

Motion Control

\\\&#x20;  ↓

Motor Driver

\\\&#x20;  ↓

Motors

\\\&#x20;  ↓

Sensors

\\\&#x20;  ↓

Perception

\\\&#x20;  ↓

Decision

```



This creates a continuous feedback loop that allows the robot to monitor its own actions and respond to changes in the environment.



\---



\## 12. Expandability



The architecture has been intentionally designed so that future subsystems can be integrated without redesigning the existing platform.



Examples include:



\- Camera

\- LiDAR

\- Raspberry Pi

\- Home Server

\- Docking Station

\- AI Coprocessor

\- Additional Sensors



\---



\## 13. Architecture Principles



The architecture follows several fundamental rules.



\- Every subsystem shall have a single primary responsibility.

\- Real-time control shall remain inside the STM32.

\- High-level processing shall remain outside the real-time control loop.

\- Communication failures shall never prevent basic robot operation.

\- Hardware upgrades should not require firmware redesign whenever possible.

\- Future expansion shall be considered during every architectural decision.



\---



\## 14. Conclusion



The ARGOS architecture separates hardware, control, perception, communication and mission management into independent layers.



This layered approach simplifies testing, encourages modular development and allows the platform to evolve from a basic mobile robot into a sophisticated autonomous security system without requiring major architectural redesign.



The architecture is intended to remain stable throughout the lifetime of the project, while individual subsystems continue to evolve.



