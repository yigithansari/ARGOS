\# ARGOS



\## Autonomous Robotic Guard \& Observation System



A long-term autonomous robotics platform developed to master embedded systems, robotics, autonomous navigation and system engineering.



\## Overview



ARGOS is a long-term engineering project focused on designing and developing a modular autonomous indoor security robot.



Unlike hobby robots, ARGOS is not intended to be a simple proof-of-concept or an Arduino-based demonstration platform. It is being developed using a professional product development methodology where every subsystem is validated before custom hardware is designed.



The project combines embedded systems, electronics, robotics, software engineering and autonomous decision-making into a single evolving platform.



The ultimate objective is to create a reliable robotic system while documenting every engineering decision throughout the development process.



\## Why ARGOS?



The primary goal of ARGOS is education through engineering.



Instead of building isolated projects, ARGOS acts as a continuously evolving robotics platform where every new subsystem extends the capabilities of the previous generation.



By the end of the project, ARGOS will demonstrate practical experience in:



\- Embedded Systems

\- STM32 Development

\- PCB Design

\- Motor Control

\- Robotics

\- Sensor Fusion

\- Autonomous Navigation

\- Battery Management

\- Communication Systems

\- Software Architecture

\- System Engineering

\- Product Development



\## Project Philosophy



ARGOS follows one fundamental engineering principle:



> Validate first. Design later.



Every subsystem is first implemented using development boards and modular hardware.



Only after validating the electrical, mechanical and software architecture will dedicated PCBs be designed.



This approach minimizes redesign effort while closely following industrial product development workflows.



\## Long-Term Vision



The final version of ARGOS is planned to become a fully autonomous indoor security robot capable of:



\- Autonomous indoor navigation

\- Patrol missions

\- Obstacle avoidance

\- Environment perception

\- Remote mission execution

\- Wireless telemetry

\- Home server integration

\- Automatic charging (future)

\- Camera-based monitoring (future)

\- Event logging

\- Future self-balancing platform



## System Architecture

                   Home Server
             (MQTT - Dashboard - Logs)
                         |
                   Wi-Fi (Future)
                         |
                Raspberry Pi / SBC
                         |
                    UART / SPI
                         |
                STM32 Main Controller
                         |
        +----------------+----------------+
        |                |                |
   Motor Control    Sensor System    Power System
        |                |                |
  Encoder Motors     IMU - ToF      Battery - BMS


\## Development Roadmap



\### Phase 0 — Bench Validation



The objective of this phase is validating all core hardware modules independently.



Topics:



\- STM32 development environment

\- PWM generation

\- Encoder reading

\- Motor driver evaluation

\- Serial debugging



\### Phase 1 — Mobile Platform



Build the first functional mobile robot.



Main goals:



\- Differential drive

\- Encoder feedback

\- PID speed control

\- Reliable movement



\### Phase 2 — Localization



Implement robot localization using onboard sensors.



Topics:



\- Wheel odometry

\- IMU integration

\- ToF distance sensing



\### Phase 3 — Basic Autonomy



Introduce autonomous behavior.



Features:



\- Obstacle avoidance

\- Basic navigation

\- Safety state machine



\### Phase 4 — Custom Electronics



Replace development boards with custom-designed hardware.



Goals:



\- STM32 custom PCB

\- Motor driver PCB

\- Power distribution

\- Professional wiring architecture



\### Phase 5 — Home Server Integration



Enable wireless communication.



Features:



\- MQTT

\- Telemetry

\- Dashboard

\- Remote mission commands



\### Phase 6 — Security Robot



Transform ARGOS into an autonomous indoor security platform.



Possible capabilities:



\- Patrol mode

\- Monitoring

\- Event detection

\- Logging

\- Camera integration



\### Phase 7 — Next Generation Platform



Potential future upgrades:



\- Self-balancing platform

\- Improved perception

\- Advanced autonomy

\- Additional robotic modules



\## Technologies



\### Embedded



\- STM32F446RE

\- STM32 HAL

\- Embedded C



\### Electronics



\- Altium Designer

\- PCB Design

\- Power Electronics



\### Robotics



\- Differential Drive

\- PID Control

\- Odometry

\- Sensor Fusion



\### Software



\- Python

\- Git

\- GitHub



\### Future Technologies



\- Raspberry Pi

\- MQTT

\- Docker

\- Home Assistant

\- Proxmox

\- OpenCV



\## Repository Structure



\- `archive/`

\- `docs/`

\- `firmware/`

\- `hardware/`

\- `images/`

\- `journal/`

\- `phases/`

\- `resources/`

\- `software/`

\- `tests/`

\- `tools/`

\- `videos/`



\## Documentation



Every engineering decision will be documented.



The repository will include:



\- Project Architecture

\- Hardware Architecture

\- Software Architecture

\- Mechanical Design

\- Test Reports

\- Development Logs

\- Lessons Learned

\- Bill of Materials (BOM)

\- Design Reviews



\## Engineering Principles



ARGOS is developed according to the following principles:



\- Modularity

\- Reliability

\- Maintainability

\- Scalability

\- Documentation First

\- Incremental Development

\- Version Control

\- Engineering Validation Before Hardware Design



\## Current Status



?? Planning Phase



The current objective is to complete the first mobile platform capable of reliable differential-drive motion using STM32, encoder feedback and PID control.



\## License



This repository is currently intended for educational and portfolio purposes.

