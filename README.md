# Magnetic Field Tracking Robot

Welcome to the code repository for the Magnetic Field Tracking Robot! This project utilizes an STM32 microcontroller to autonomously navigate a path using magnetic field sensors, backed by a robust PID control loop. It also includes comprehensive collision avoidance, infrared remote control capabilities, and computer vision integration via a Pixy2 camera over Bluetooth (BLE).

## 🚀 Key Features

*   **Autonomous Tracking & Navigation**: Reads dual analog magnetic sensors using ADC and keeps the robot centered on the path using a custom differential-steering PID controller.
*   **Intelligent Intersection Handling**: Uses a third center ADC sensor to detect intersections and follows pre-programmed path sequences (modes) automatically.
*   **Auto-Recovery Mode**: If the robot drifts off the track (and all sensors drop below threshold), it performs an automated sweeping spin to relocate the field and continue.
*   **Collision Avoidance (ToF)**: Integrates the `VL53L0X` Time-of-Flight distance sensor via I2C to instantly stop if an obstacle is detected within 95mm.
*   **IR Remote Command System**: Custom timer-interrupt-based IR decoder and transmitter. Responds to encoded pulse-width signals for manual driving, mode cycling, and stopping.
*   **Vision Decoding (PC/Host GUI)**: Includes `receiver.py`, an asynchronous Python application using `bleak` and `tkinter`, that syncs with a JDY-23 BLE module to parse data packets from a Pixy2 Camera (detecting colored blocks and calculating coordinates). 

## 🛠️ Tech Stack & Hardware Components

### Software & Firmware
*   **Language**: C (Embedded Firmware), Python (Host GUI)
*   **MCU Core**: ARM Cortex-M0+ (STM32L051xx)
*   **Toolchain**: `arm-none-eabi-gcc` / Makefiles
*   **Control Loop**: Proportional-Integral-Derivative (PID) algorithm
*   **Python Libraries**: `bleak` (Async BLE), `tkinter` (GUI Canvas), `struct`, `asyncio`

### Hardware Modules
*   **Microcontroller**: STM32L051xx LQFP32
*   **Sensors**: Magnetic Field Sensors (Analog), VL53L0X (I2C ToF Distance) 
*   **Motors**: DC Motors driven by custom PWM signals (via TIM2 hardware timer).
*   **Comm Modules**: Infrared (IR) Receiver/Transmitter, JDY-23 BLE Module, Pixy2 Vision Camera.

## 📁 Repository Structure

*   **`main.c`**: The core finite state machine (FSM). Handles the main loop, IR command decoding, PID calculations, auto-recovery logic, and sensor polling.
*   **`motor.c` / `motor.h`**: Configures the STM32 `TIM2` timer for PWM generation and maps high-level directional commands (e.g., `robotForward`, `turnLeft`) alongside dynamic differential `Motor_SetPWM` inputs.
*   **`vl53l0x.c` / `vl53l0x.h`**: Custom I2C driver for initializing and polling distance data from the Time-of-Flight sensor.
*   **`ADC.c` / `ADC.h`**: Hardware configuration for analog-to-digital conversions, reading the multiple tracking sensors.
*   **`decoder.c` / `robot_ir_tx.c`**: Interrupt-driven signal measurement code to capture pulse widths and execute user remote commands.
*   **`receiver.py`**: A desktop-side Python application that parses fragmented BLE packets for Pixy2 signature blocks and visually plots them to a GUI in real-time.
*   **`stm32l051xx.h` / `Common/`**: Device-specific headers and CMSIS core registers definition.

## ⚙️ Compilation & Deployment

This project uses standard Makefiles. 
1. Ensure you have the `arm-none-eabi` toolchain installed.
2. Build the `.hex` file using `make -f project2.mk` or `make -f makefile.mac`.
3. Flash the generated executable to the STM32 via ST-Link or an integrated serial bootloader.
