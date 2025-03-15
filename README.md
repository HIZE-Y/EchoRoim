# Trilobot Example Applications
 
> **Important:** Please refer to the README in the `/common` directory for detailed build instructions and library configuration before exploring these examples.
 
## Table of Contents
 
- [Overview](#overview)
- [Application Examples](#application-examples)
  - [button_rgb.c](#button_rgcc)
  - [ultrasonic.c](#ultrasonicc)
  - [motor_sw_pwm.c](#motor_sw_pwmc)
  - [rgb_led.c](#rgb_ledc)
  - [imu.c](#imuc)
- [Usage ](#usage)
- [Final Notes](#final-notes)
 
## Overview
 
The Trilobot platform provides a unified API (declared in `trilobot.h`) for interacting with various hardware components on a Raspberry Pi–based system. This public API covers:
 
- **Motor Control:** Uses software PWM to drive motors.
- **IMU (ICM‑20948):** Provides functions to initialize, calibrate, and read the IMU sensor data.
- **Ultrasonic Sensor:** Enables distance measurement via a trigger and echo pins.
- **SN3218 LED Driver:** Controls RGB LEDs by managing individual LED channels and displaying test patterns.
- **GPIO Utilities:** Facilitates button input and LED output control.
- **LED Animation & Button Sequence:** Offers routines for recording button sequences and running LED animations.
 
The examples below demonstrate how to use these APIs in individual application files.
 
## Application Examples
 
### button_rgb.c
 
- **Purpose:**  
  Demonstrates the LED Animation & Button Sequence API. This example initializes the Trilobot system, configures button inputs and LED outputs, and continuously monitors button events. It records button sequences and triggers corresponding LED animations or test patterns based on recognized patterns.
 
- **How to Use:**  
  Compile and run this example to observe real-time LED feedback in response to button presses. 
 
---
 
### ultrasonic.c
 
- **Purpose:**  
  Demonstrates the Ultrasonic Sensor API. This example initializes the sensor, sends trigger pulses, and measures the distance by calculating the time between sending the pulse and receiving the echo.
 
- **How to Use:**  
  Run this example to obtain distance measurements (in centimeters) from the ultrasonic sensor. 
 
---
 
### motor_sw_pwm.c
 
- **Purpose:**  
  Demonstrates the Motor Driver API utilizing software PWM. This example shows how to initialize motor control, set speeds, change directions (forward and reverse), and safely stop the motors. It includes a SIGINT signal handler to disable the motors when terminating the program.
 
- **How to Use:**  
  Use this example to test motor operations such as starting, stopping, and pivot turning.

---
 
### rgb_led.c
 
- **Purpose:**  
  Demonstrates the SN3218 LED Driver API for controlling RGB LEDs. This example initializes the LED driver, sets brightness levels for individual LED channels, and displays test patterns including a channel-walk test and RGB color cycles.
 
- **How to Use:**  
  Run this example to validate LED control functionality. 
 
---
 
### imu.c
 
- **Purpose:**  
  Demonstrates the IMU (ICM‑20948) API. This example initializes the IMU sensor, performs a calibration routine to compute sensor offsets, and continuously reads raw accelerometer and gyroscope data from the sensor.
 
- **How to Use:**  
  Execute this example to monitor sensor data output on the console. 
 
---
 
## Usage
 
- **Hardware Connections:**  
  Ensure that all components—motors, ultrasonic sensor, LEDs, buttons, and the IMU—are connected according to the GPIO and I²C configurations defined in `trilobot.h`.
 
- **Compilation:**  
  Each example relies on the Trilobot API and the common hardware libraries. Use the provided Makefiles (and follow the build instructions in the `/common` README) to compile these examples. Running the examples may require root privileges due to direct hardware access.


