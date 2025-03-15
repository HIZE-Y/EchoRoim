/******************************************************************************
* Copyright (c) 2025, BlackBerry Limited. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* @file trilobot.h
* @brief Public API for the Trilobot library.
*
* This library provides functions to control the Trilobot hardware including:
*  - Motor driver using software PWM.
*  - SparkFun ICM‑20948 9DoF IMU.
*  - HC‑SR04 Ultrasonic Sensor.
*  - SN3218 LED driver for RGB LED control.
*  - Basic GPIO utilities for buttons and LEDs.
*  - LED Animation & Button Sequence API.
*
*****************************************************************************/
 
#ifndef TRILOBOT_H
#define TRILOBOT_H
 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include "rpi_gpio.h"
#include "rpi_i2c.h"
 
/* Return Codes */
#define SUCCESS 0
#define FAILURE -1
 
/* Global pointer for accessing GPIO registers */
extern volatile uint32_t *__RPI_GPIO_REGS;
 
/*-----------------------------------------------------------
* Public Definitions (Hardware Constants)
*-----------------------------------------------------------
*/
 
// Motor GPIO pin definitions.
#define MOTOR_EN_PIN    26   // Motor enable pin (SW PWM controlled)
#define MOTOR_RP_PIN    10   // Right Motor "Forward" input.
#define MOTOR_RN_PIN    9    // Right Motor "Reverse" input.
#define MOTOR_LP_PIN    8    // Left Motor "Forward" input.
#define MOTOR_LN_PIN    11   // Left Motor "Reverse" input.
 
// Ultrasonic sensor GPIO pin definitions.
#define GPIO_PULSE_PIN      13   // Trigger/pulse pin.
#define GPIO_INTERRUPT_PIN  25   // Echo pin.
 
// LED and Button enumerations.
enum Leds {
    Led_A = 23,
    Led_B = 22,
    Led_X = 17,
    Led_Y = 27
};
 
enum Buttons {
    Button_A = 5,
    Button_B = 6,
    Button_X = 16,
    Button_Y = 24
};
 
/*-----------------------------------------------------------
* Bot Initialization and Basic GPIO Functions
*-----------------------------------------------------------
*/
 
/**
* @brief Maps the GPIO registers and initializes the bot.
*
* @return int SUCCESS on success, FAILURE on failure.
*/
int initialize_bot(void);
 
/**
* @brief Initializes a button GPIO as an input with pull-up.
*
* @param button The GPIO pin number for the button.
* @return int SUCCESS on success, FAILURE on failure.
*/
int initialize_button(uint32_t button);
 
/**
* @brief Reads the state of a button.
*
* @param button The GPIO pin number for the button.
* @return int The button state (0 or 1).
*/
int read_button(uint32_t button);
 
/**
* @brief Initializes an LED GPIO for output.
*
* @param led The GPIO pin number for the LED.
* @return int SUCCESS on success.
*/
int initialize_led(uint32_t led);
 
/**
* @brief Sets the state of an LED.
*
* @param led The GPIO pin number for the LED.
* @param state 0 for LOW, 1 for HIGH.
* @return int SUCCESS on success.
*/
int set_led(uint32_t led, uint32_t state);
 
/*-----------------------------------------------------------
* Ultrasonic Sensor API
*-----------------------------------------------------------
*/
 
/**
* @brief Initializes the ultrasonic sensor GPIOs.
*
* Configures the trigger (GPIO_PULSE_PIN) and echo (GPIO_INTERRUPT_PIN) pins.
*
* @return int SUCCESS on success, FAILURE on error.
*/
int ultrasonic_init(void);
 
/**
* @brief Reads the distance measured by the ultrasonic sensor.
*
* Sends a trigger pulse and calculates the distance (in cm) from the echo.
*
* @param distance Pointer to store the computed distance.
* @return int SUCCESS on success, FAILURE on error.
*/
int ultrasonic_read_distance(float *distance);
 
/*-----------------------------------------------------------
* SN3218 LED Driver API
*-----------------------------------------------------------*/

// Controls an 18-channel LED driver for 6 RGB LEDs.


/* --- SN3218 LED Driver Definitions --- */
#define I2C_BUS            1
#define SN3218_ADDR        0x54
#define REG_SHUTDOWN       0x00
#define REG_PWM_BASE       0x01
#define REG_ENABLE_LEDS    0x13
#define REG_UPDATE         0x16
#define REG_RESET          0x17
#define SN3218_NUM_CHANNELS 18
 
/**
* @brief Initializes the SN3218 LED driver.
*
* @return int SUCCESS on success, FAILURE on error.
*/
int trilobot_led_init(void);
 
/**
* @brief Sets the brightness for a logical LED channel.
*
* @param channel Logical channel index (0..17).
* @param brightness Brightness value (0..255).
*/
void trilobot_led_set_channel(uint8_t channel, uint8_t brightness);
 
/**
* @brief Sets an RGB LED to a specified color.
*
* @param led_index LED index (0..5).
* @param r Red brightness.
* @param g Green brightness.
* @param b Blue brightness.
*/
void trilobot_led_set_rgb(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);
 
/**
* @brief Updates the LED driver with the current brightness values.
*
* @return int SUCCESS on success, FAILURE on error.
*/
int trilobot_led_update(void);
 
/**
* @brief Displays a test pattern on the RGB LEDs.
*/
void trilobot_led_test_pattern(void);
 
/**
* @brief Runs a channel-walk test to verify LED wiring.
*/
void trilobot_led_channel_walk_test(void);

/*-----------------------------------------------------------
* LED Animation & Button Sequence API
*-----------------------------------------------------------
*/
 
/**
* @brief Resets the internal button sequence buffer.
*/
void trilobot_reset_button_sequence(void);
 
/**
* @brief Records a button event in the internal sequence buffer.
*
* @param btn The button character ('A', 'B', 'X', or 'Y').
*/
void trilobot_record_button(char btn);
 
/**
* @brief Checks if the internal button sequence matches a special pattern.
*
* Currently, if the sequence is "AABB" or "XXYY", returns true.
*
* @return bool true if special sequence detected; false otherwise.
*/
bool trilobot_check_special_sequence(void);
 
/**
* @brief Runs an RGB loop animation.
*
* This animation cycles through red, green, blue, and off.
*/
void trilobot_run_rgb_loop(void);

/**
* @brief Gets the current button sequence.
*
* @return const char* The current button sequence.
*/
const char* trilobot_get_button_sequence(void);
 
/*-----------------------------------------------------------
* Motor Driver API (Software PWM)
*-----------------------------------------------------------
*/

/* --- Motor SW PWM Definitions --- */
#define PWM_FREQ 100  // Hz
 

/**
* @brief Initializes the motor driver hardware.
*
* Configures the enable pin for software PWM and sets up the direction pins.
*
* @return int SUCCESS on success, FAILURE on error.
*/
int motor_init(void);
 
/**
* @brief Sets the motor speed by updating the SW PWM duty cycle.
*
* @param speed_percent Desired speed as a percentage (0.0 to 100.0).
* @return int SUCCESS on success.
*/
int motor_set_speed(float speed_percent);
 
/**
* @brief Disables the motor driver (sets speed to 0%).
*
* @return int SUCCESS on success.
*/
int motor_disable(void);
 
/**
* @brief Sets the right motor to drive forward.
*
* @return int SUCCESS on success.
*/
int motor_right_forward(void);
 
/**
* @brief Sets the right motor to drive in reverse.
*
* @return int SUCCESS on success.
*/
int motor_right_reverse(void);
 
/**
* @brief Sets the left motor to drive forward.
*
* @return int SUCCESS on success.
*/
int motor_left_forward(void);
 
/**
* @brief Sets the left motor to drive in reverse.
*
* @return int SUCCESS on success.
*/
int motor_left_reverse(void);
 
/**
* @brief Stops both motors.
*
* @return int SUCCESS on success.
*/
int motor_stop(void);
 
/**
* @brief Executes a pivot turn to the left.
*
* @return int SUCCESS on success.
*/
int motor_turn_left(void);
 
/**
* @brief Executes a pivot turn to the right.
*
* @return int SUCCESS on success.
*/
int motor_turn_right(void);
 
/**
* @brief Signal handler for SIGINT to safely disable motors.
*
* @param sig Signal number.
*/
void motor_sigint_handler(int sig);
 
/*-----------------------------------------------------------
* IMU (ICM‑20948) API
*-----------------------------------------------------------
*/
#define ICM20948_ADDR   0x69
#define REG_BANK_SEL    0x7F
#define BANK_0          0x00
#define BANK_2          0x20
#define WHO_AM_I        0x00      // Expected: 0xEA
#define PWR_MGMT_1      0x06
#define PWR_MGMT_2      0x07
#define ACCEL_CONFIG    0x14      // ±4g
#define GYRO_CONFIG_1   0x01      // ±500 dps
#define ACCEL_XOUT_H    0x2D
 
#define ACCEL_SENSITIVITY   8192.0
#define GYRO_SENSITIVITY    65.5

#define CALIBRATION_SAMPLES 100
#define ALPHA 0.1
#define LOOP_DELAY_US 20000
/**
* @brief Initializes the ICM‑20948 IMU.
*
* Wakes and configures the sensor.
*
* @return int SUCCESS on success, FAILURE on error.
*/
int imu_init(void);
 
/**
* @brief Calibrates the IMU.
*
* Averages a specified number of samples to compute sensor offsets.
*
* @param num_samples Number of samples for calibration.
* @param accel_offset_x Pointer to store accelerometer X offset (g).
* @param accel_offset_y Pointer to store accelerometer Y offset (g).
* @param accel_offset_z Pointer to store accelerometer Z offset (g).
* @param gyro_offset_x Pointer to store gyroscope X offset (°/s).
* @param gyro_offset_y Pointer to store gyroscope Y offset (°/s).
* @param gyro_offset_z Pointer to store gyroscope Z offset (°/s).
* @return int SUCCESS on success, FAILURE on error.
*/
int imu_calibrate(int num_samples,
                  double *accel_offset_x, double *accel_offset_y, double *accel_offset_z,
                  double *gyro_offset_x, double *gyro_offset_y, double *gyro_offset_z);
 
/**
* @brief Reads raw sensor data from the IMU.
*
* @param ax Pointer to store accelerometer X reading.
* @param ay Pointer to store accelerometer Y reading.
* @param az Pointer to store accelerometer Z reading.
* @param gx Pointer to store gyroscope X reading.
* @param gy Pointer to store gyroscope Y reading.
* @param gz Pointer to store gyroscope Z reading.
* @return int SUCCESS on success, FAILURE on error.
*/
int imu_read_sensors(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
 
#endif // TRILOBOT_H
