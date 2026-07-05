\# ARGOS System Requirements



\*\*Document ID:\*\* DOC-001

\*\*Document Title:\*\* System Requirements Specification (SRS)

\*\*Version:\*\* 1.0 (Draft)

\*\*Status:\*\* Active

\*\*Project:\*\* ARGOS – Autonomous Robotic Guard \& Observation System



\---



\## 1. Purpose



This document defines the engineering requirements for the ARGOS platform.



It serves as the primary reference for every future design decision regarding hardware, software, mechanical design, electronics, firmware, testing and validation.



Any subsystem developed for ARGOS should satisfy the requirements defined in this document.



Whenever a design decision conflicts with these requirements, this document shall take precedence unless officially revised.



\---



\## 2. Scope



ARGOS is an autonomous indoor robotic platform intended to serve as a long-term engineering and learning project.



The platform is designed to evolve through multiple generations while maintaining backward compatibility whenever practical.



The first generation focuses on establishing a reliable robotic foundation.



Future generations will introduce autonomy, wireless communication, custom electronics, environmental perception and security-related capabilities.



\---



\## 3. System Overview



The complete ARGOS platform consists of several independent but interconnected subsystems.



These include:



\- Mechanical Platform

\- Motion System

\- Embedded Control System

\- Power Distribution System

\- Sensor System

\- Communication System

\- Navigation System

\- Software Infrastructure

\- Home Server Integration (Future)



Each subsystem shall be independently testable before integration.



\---



\## 4. Functional Requirements



The robot shall be capable of:



\- Moving forward.

\- Moving backward.

\- Rotating in place.

\- Turning with differential drive.

\- Measuring wheel rotation.

\- Estimating wheel speed.

\- Reading environmental sensors.

\- Monitoring battery voltage.

\- Executing movement commands.

\- Entering safe stop mode.

\- Supporting future autonomous navigation.



The robot shall be designed so that every capability can be extended without redesigning the entire platform.



\---



\## 5. Mechanical Requirements



\### 5.1 Size



Target dimensions:



\- Approximately shoe-box sized.



The robot should remain compact enough for indoor navigation.



\---



\### 5.2 Weight



Phase 1 target:



The robot shall only carry:



\- Battery

\- STM32 controller

\- Sensors

\- Motor driver

\- Wiring



No additional payload is required.



Future payload capacity should be considered during chassis design.



\---



\### 5.3 Drive System



The robot shall use:



\- Differential drive



Configuration:



\- Two powered wheels

\- One caster wheel



This configuration provides:



\- Simple kinematics

\- Easy control

\- Good maneuverability

\- Future migration path toward self-balancing architecture



\---



\### 5.4 Maximum Speed



The maximum speed shall remain below normal human running speed.



Phase 1 prioritizes stability over speed.



\---



\### 5.5 Runtime



Minimum target:



20–25 minutes of continuous operation.



Future revisions may increase runtime through improved battery capacity and power optimization.



\---



\## 6. Electrical Requirements



The embedded controller shall provide:



\- Multiple PWM outputs

\- Quadrature encoder interfaces

\- ADC channels

\- UART communication

\- SPI interfaces

\- I²C interfaces

\- Debug capability

\- SWD programming



Initial controller:



\- STM32 NUCLEO-F446RE



Future controller:



\- Custom STM32 PCB.



\---



\## 7. Power Requirements



Power architecture shall support:



\- Battery operation

\- Motor power

\- Logic power

\- Sensor power



Power system shall include:



\- Main power switch

\- Battery protection

\- Voltage regulation

\- Common ground architecture



Future revisions shall include:



\- Battery health monitoring

\- Charging management

\- Docking support



\---



\## 8. Motion Requirements



Motion system shall support:



\- Independent wheel control

\- Encoder feedback

\- Closed-loop speed control

\- PID regulation

\- Smooth acceleration

\- Smooth deceleration



Future support:



\- Motion profiling

\- Trajectory following

\- Self-balancing



\---



\## 9. Sensor Requirements



Phase 1 sensors:



Mandatory:



\- Wheel encoders

\- IMU

\- ToF distance sensors



Future sensors:



\- Camera

\- LiDAR

\- Docking sensors

\- Microphone

\- Environmental sensors



The architecture shall allow additional sensors without major redesign.



\---



\## 10. Communication Requirements



Phase 1:



\- USB

\- UART



Future:



\- Wi-Fi

\- MQTT

\- Home Server

\- Dashboard

\- Remote diagnostics



The communication layer shall remain modular.



\---



\## 11. Safety Requirements



The robot shall include:



\- Emergency stop capability.

\- Battery voltage monitoring.

\- Safe startup.

\- Safe shutdown.

\- Motor timeout detection.

\- Watchdog recovery.



The robot shall never continue moving after a critical failure.



Safety always has higher priority than mission execution.



\---



\## 12. Software Requirements



Firmware shall be modular.



Modules should include:



\- Motor Driver

\- Encoder Driver

\- PID Controller

\- Sensor Manager

\- Communication Manager

\- Safety Manager

\- Diagnostics



Future software shall support RTOS if required.



\---



\## 13. Documentation Requirements



Every subsystem shall include:



\- Purpose

\- Architecture

\- Interfaces

\- Testing procedure

\- Known limitations

\- Future improvements

\- Engineering decisions



The project shall maintain complete engineering documentation throughout its lifecycle.



\---



\## 14. Testing Requirements



Every subsystem shall be validated independently before integration.



Each test shall include:



\- Objective

\- Equipment

\- Procedure

\- Expected Result

\- Actual Result

\- Conclusion



Test videos should accompany major milestones whenever possible.



\---



\## 15. Phase 1 Minimum Requirements



Phase 1 shall be considered complete when ARGOS can:



\- Move forward

\- Move backward

\- Rotate

\- Control both motors independently

\- Read both wheel encoders

\- Maintain wheel speed using PID

\- Report debug information over UART

\- Operate reliably for at least 20 minutes



\---



\## 16. Future Requirements



The following capabilities are intentionally excluded from Phase 1:



\- Custom PCB

\- Raspberry Pi

\- ROS

\- Camera

\- LiDAR

\- Wi-Fi

\- MQTT

\- Home Server

\- Autonomous navigation

\- Docking station

\- Self-balancing



These features shall be introduced only after the mobile platform has been fully validated.



\---



\## 17. Requirement Priorities



| Priority | Description |

|---|---|

| Critical | Reliable motion platform |

| Critical | Stable power system |

| Critical | Encoder feedback |

| Critical | PID speed control |

| High | Sensor integration |

| High | Modular firmware |

| Medium | Expandable hardware |

| Medium | Professional documentation |

| Low (Phase 1) | Wireless communication |

| Low (Phase 1) | Camera |

| Low (Phase 1) | AI integration |



\---



\## 18. Engineering Acceptance Criteria



ARGOS shall be considered a successful engineering platform if it satisfies the following principles:



\- Every subsystem is independently testable.

\- Hardware is modular and maintainable.

\- Software is reusable and scalable.

\- Documentation is sufficient for future redesigns.

\- Every design decision is supported by engineering reasoning.

\- The platform remains expandable for multiple future generations.



Meeting these criteria is considered more important than implementing a large number of features.



\---



\## Revision History



| Version | Description |

|---|---|

| 1.0 | Initial system requirements specification. |



