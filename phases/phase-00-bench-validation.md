\# Phase 00 – Bench Validation



\*\*Document ID:\*\* PHASE-000  

\*\*Document Title:\*\* Phase 00 – Bench Validation  

\*\*Version:\*\* 1.0 (Draft)  

\*\*Status:\*\* Planning  

\*\*Project:\*\* ARGOS – Autonomous Robotic Guard \& Observation System



\---



\## 1. Purpose



Phase 00 is the foundation of the ARGOS project.



The objective of this phase is to validate every critical hardware and firmware subsystem individually before integrating them into a mobile robotic platform.



No chassis is required during this phase.



The focus is entirely on understanding, testing and validating each subsystem in a controlled bench environment.



This phase significantly reduces integration problems during future development.



\---



\## 2. Objectives



The primary objectives of Phase 00 are:



\- Establish the embedded development environment.

\- Validate all selected hardware components.

\- Develop reusable firmware modules.

\- Verify communication interfaces.

\- Understand the behavior of every subsystem.

\- Create reliable test procedures.

\- Build confidence before mechanical integration.



\---



\## 3. Learning Goals



By the end of Phase 00, the following engineering topics should be understood:



\- STM32 development workflow

\- STM32CubeIDE

\- GPIO configuration

\- Timer peripherals

\- PWM generation

\- Quadrature encoder interface

\- UART debugging

\- I²C communication

\- Sensor interfacing

\- Basic motor control

\- PID fundamentals

\- Embedded debugging techniques

\- Git \& GitHub workflow



\---



\## 4. Required Hardware



The following hardware is expected during this phase.



\### Main Controller



\- STM32 NUCLEO-F446RE



\### Motion System



\- 2 × Encoder DC Motors

\- Motor Driver



\### Sensors



\- MPU6050

\- VL53L0X or equivalent



\### Power



\- Bench Power Supply, preferred



or



\- Temporary battery solution



\### Miscellaneous



\- Breadboard

\- Jumper wires

\- USB cable

\- Logic analyzer, optional

\- Multimeter



\---



\## 5. Firmware Modules



During Phase 00 the firmware shall be divided into reusable modules.



Modules include:



\- GPIO Driver

\- PWM Driver

\- Encoder Driver

\- UART Driver

\- I²C Driver

\- Motor Driver

\- IMU Driver

\- ToF Driver

\- PID Controller

\- Debug Utilities



Each module should be independently testable.



\---



\## 6. Validation Tasks



The following tasks shall be completed.



\### Task 01



STM32 development environment



Success Criteria:



\- Project builds successfully.

\- Firmware uploads successfully.

\- Debugger operates correctly.



\---



\### Task 02



LED Blink



Purpose:



Verify basic firmware operation.



\---



\### Task 03



UART Communication



Verify:



\- Serial output

\- Command reception

\- Debug messages



\---



\### Task 04



PWM Generation



Verify:



\- Frequency

\- Duty cycle

\- Stable output



\---



\### Task 05



Motor Driver



Verify:



\- Direction control

\- Enable control

\- Safe startup

\- Safe shutdown



\---



\### Task 06



Encoder Interface



Verify:



\- Pulse counting

\- Direction detection

\- RPM calculation



\---



\### Task 07



IMU



Verify:



\- Accelerometer

\- Gyroscope

\- Raw sensor data



\---



\### Task 08



ToF Sensor



Verify:



\- Distance measurements

\- Stable readings

\- Communication reliability



\---



\### Task 09



PID Preparation



Implement:



\- PID structure

\- Test interface

\- Debug variables



Complete tuning is intentionally postponed until Phase 01.



\---



\## 7. Deliverables



The following deliverables are expected.



\### Hardware



\- Working bench setup



\### Firmware



\- Modular firmware architecture

\- Reusable drivers



\### Documentation



\- Wiring diagrams

\- Test notes

\- GitHub commits

\- Development log



\---



\## 8. Risks



Potential risks include:



\- Incorrect motor selection

\- Encoder incompatibility

\- Sensor communication issues

\- Wiring mistakes

\- Power instability



Every identified issue should be documented before entering Phase 01.



\---



\## 9. Exit Criteria



Phase 00 shall be considered complete only if all of the following conditions are satisfied:



\- STM32 development environment is fully operational.

\- PWM generation has been verified.

\- Motor driver functions correctly.

\- Encoder data is reliable.

\- IMU communication is stable.

\- ToF sensor is operational.

\- UART debugging is functional.

\- Firmware modules are organized and reusable.

\- All validation tests have been documented.



No chassis integration shall begin before these conditions are met.



\---



\## 10. Phase Certification



A Phase 00 Certified ARGOS Platform is defined as a hardware validation platform where every critical subsystem has been individually verified and documented.



The platform is now considered ready for mechanical integration and the construction of the first mobile robotic prototype in Phase 01.



\---



\## 11. Completion Statement



Phase 00 is not about building a robot.



It is about building confidence.



By validating every subsystem independently, Phase 00 establishes the technical foundation upon which every future generation of ARGOS will be developed.

