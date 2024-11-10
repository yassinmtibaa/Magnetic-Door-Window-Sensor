# Magnetic Door/Window Sensor System

## Overview

This project implements a simple security system to detect whether a door or window is open or closed using a **Magnetic Reed Switch** and an **STM32F0 microcontroller**. When the door or window is opened, the sensor breaks the circuit, triggering an action (e.g., activating a buzzer and LED). The system can be further enhanced with wireless notifications, alarms, or other features.

### Key Features:
- **Reed Switch** for door/window status detection.
- **STM32F0** microcontroller for processing sensor input.
- **Buzzer** for audible alerts when the door/window is opened.
- **LED** to indicate the current status of the door/window.
- Optional upgrades: Wireless notifications, battery management, and enhanced security features.

## Components Required

- **Reed Switch**: Detects the presence of a magnet and triggers the system when the door/window is opened.
- **Magnet**: Attached to the movable part of the door/window.
- **STM32F0 Microcontroller**: Processes input from the reed switch and controls the buzzer and LED.
- **Buzzer**: Provides an audio signal when the door/window is opened.
- **LED**: Visual indicator for open/closed state of the door/window.
- **Power Supply**: Battery or external source to power the system.

## System Requirements

- **STM32F0 Discovery Board**
- **STM32CubeIDE** and **STM32CubeMX** for development and configuration
- **CMake** for building the project (if using custom build setup)

## Project Setup

### Step 1: Configure the STM32F0 using STM32CubeMX

1. Open **STM32CubeMX** and create a new project for the **STM32F0** microcontroller.
2. **Configure GPIO pins**:
   - **PA0** (Reed Switch) as input with pull-up resistor.
   - **PA1** (Buzzer) as output (push-pull).
   - **PA2** (LED) as output (push-pull).
3. Set the **system clock** to use the internal HSI oscillator.
4. **Generate the code** for the STM32F0 using STM32CubeMX, then open the project in **STM32CubeIDE**.

### Step 2: Clone the Repository

Clone the repository to your local machine:

```bash
git clone https://github.com/yassinmtibaa/magnetic-door-sensor.git
cd magnetic-door-sensor
