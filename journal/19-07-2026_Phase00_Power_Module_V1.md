# Phase 00 - Power Module V1

**Date:** 2026-07-19

## Summary

The first version of the ARGOS power distribution module was assembled on a perfboard as part of Phase 00.

The objective of this module is to provide a reliable and reusable power distribution solution for bench validation before designing a custom PCB.

---

## Implemented Power Architecture

The module consists of:

- 3S Li-ion battery pack
- 3S Battery Management System (BMS)
- Main power switch
- 5A blade fuse
- 12V output header
- LM2596 buck converter
- 5V output header

The battery output passes through the BMS before reaching the main power switch.

After the switch, power is protected by a 5A blade fuse.

The fused 12V rail is distributed directly to the motor drivers while simultaneously feeding the LM2596 buck converter.

The LM2596 generates a regulated 5V supply for the STM32 NUCLEO board and low-voltage peripherals.

---

## Purpose

This module establishes a reusable power distribution platform for Phase 00 hardware validation.

It allows every subsystem to share a common power source while remaining protected by a main fuse and switch.

---

## Documentation

Related files:

- images/diagrams/Phase00_System_Block_Diagram.png
- images/hardware/2026-07-19_Power_Module_01.jpg
- images/hardware/2026-07-19_Power_Module_02.jpg

---

## Lessons Learned

Building the power distribution module before assembling the complete robot simplifies future hardware testing and reduces repeated wiring during subsystem validation.

---

## Current Phase

Phase 00 - Bench Validation

---

## Next Milestone

Validate the power module under load and use it as the primary power source for the STM32, motor drivers and sensors during bench testing.
