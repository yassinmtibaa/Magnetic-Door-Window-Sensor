# Magnetic Door/Window Sensor System

## Overview

This project implements a simple security system to detect whether a door or window is open or closed using a **Magnetic Reed Switch** and an **STM32F0 microcontroller**. When the door or window is opened, the sensor breaks the circuit, triggering an action (e.g., activating a buzzer and LED). The system can be further enhanced with wireless notifications, alarms, or other features.

### Key Features:
- **Reed Switch** for door/window status detection.
- **STM32F0** microcontroller for processing sensor input.
- **Buzzer** for audible alerts when the door/window is opened.
- **LED** to indicate the current status of the door/window.
- **LCD** to dsiplay the status of the system.
- Optional upgrades: Wireless notifications, battery management, and enhanced security features.

## Components Required

- **Reed Switch**: Detects the presence of a magnet and triggers the system when the door/window is opened.
- **Magnet**: Attached to the movable part of the door/window.
- **STM32F0 Microcontroller**: Processes input from the reed switch and controls the buzzer and LED.
- **Buzzer**: Provides an audio signal when the door/window is opened.
- **LED**: Visual indicator for open/closed state of the door/window.
- **LCD** to dsiplay the status of the system.
- **Power Supply**: Battery or external source to power the system.

## System Requirements

- **STM32F0 Discovery Board**
- **STM32CubeIDE** and **STM32CubeMX** for development and configuration
- **CMake** for building the project (if using custom build setup)

## Project Setup

### Step 1: Configure the STM32F0 using STM32CubeMX

1. Open **STM32CubeMX** and create a new project for the **STM32F0** microcontroller.
2. **Configure GPIO pins**:
   - **PB5** (Reed Switch) as input with pull-up resistor.
   - **PB6** (green LED) as output (push-pull).
   - **PB7** (red LED) as output (push-pull).
   - **PB15** (Buzzer) as output (push-pull).
   - **PB10** I2C2-SCL (LCD).
   - **PB11** I2C2-SDA (LCD).
3. Set the **system clock** to use the internal HSI oscillator.
4. **Generate the code** for the STM32F0 using STM32CubeMX, then open the project in **STM32CubeIDE**.

### Step 2: Clone the Repository

Clone the repository to your local machine:

```bash
git clone https://github.com/yassinmtibaa/magnetic-door-sensor.git
cd magnetic-door-sensor
```
### Step 3: Build and Program the STM32F0
1. Open STM32CubeIDE and load the project.
2. Build the project by clicking Project > Build Project.
3. Program the STM32F0 microcontroller via the ST-Link programmer.
4. Ensure that the reed switch, buzzer, and LED are connected correctly to the STM32F0 pins (PA0, PA1, and PA2).


### Step 4: Testing the System
- When the door or window is closed, the reed switch will be in a closed state (magnet present), and the system should turn off the buzzer and LED (green) and print on the display system is on.
- When the door or window is opened, the reed switch will open (magnet moved away), and the system will trigger the buzzer and turn on the LED (red) and print on the display the system is off .

## Code Explanation

### Main Components:

- **main.c:** This file contains the main logic for reading the reed switch status and controlling the buzzer and LED.
- **GPIO Initialization (MX_GPIO_Init):** Configures the GPIO pins for input and output.
- **Reed Switch Logic:** Checks the status of the reed switch (open or closed) and takes appropriate action (activating the buzzer and LED).
- **System Clock Configuration:** Ensures the STM32F0 microcontroller operates at the correct clock speed.

## Example Code:
```bash
#include "main.h"
#include "stm32f0xx_hal.h"

#define DOOR_SENSOR_PIN GPIO_PIN_0  // Reed Switch connected to PA0
#define BUZZER_PIN GPIO_PIN_1       // Buzzer connected to PA1
#define LED_PIN GPIO_PIN_2          // LED connected to PA2

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        // Check the status of the reed switch (door/window status)
        if (HAL_GPIO_ReadPin(GPIOA, DOOR_SENSOR_PIN) == GPIO_PIN_RESET)
        {
            // Door/window is open, activate buzzer and LED
            HAL_GPIO_WritePin(GPIOA, BUZZER_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_SET);  // Turn on red LED
        }
        else
        {
            // Door/window is closed, deactivate buzzer and LED
            HAL_GPIO_WritePin(GPIOA, BUZZER_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, LED_PIN, GPIO_PIN_RESET); // Turn off LED
        }

        HAL_Delay(100);
    }
}
```
## Contributing
If you would like to contribute to this project, feel free to fork the repository, make changes, and create a pull request.

### Steps to Contribute:
1. Fork the repository to your GitHub account.
2. Clone your fork to your local machine.
3. Make changes to the code, documentation, or hardware setup.
4. Commit your changes and push them to your fork.
5. Open a pull request to merge your changes back into the main repository.


## Acknowledgements
STM32CubeMX and STM32CubeIDE for making development easier with HAL libraries.
STM32F0 Discovery Board for providing a low-cost and powerful development platform.


