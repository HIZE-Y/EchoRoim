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
* @file trilobot.c
* @brief Implementation of the Trilobot library.
*
* This library provides functions to control the Trilobot hardware including:
*  - Motor driver using software PWM.
*  - SparkFun ICM‑20948 9DoF IMU.
*  - HC‑SR04 Ultrasonic Sensor.
*  - SN3218 LED driver for RGB LED control.
*  - Basic GPIO utilities for buttons and LEDs.
*  - LED Animation & Button Sequence API.
*
**************************************************************************/
 
#include "trilobot.h"
#include <string.h>

 
/*===========================================================================
* Global Variables
*===========================================================================*/
 
volatile uint32_t *__RPI_GPIO_REGS = NULL;
 
/*===========================================================================
* Bot Initialization and Basic GPIO Functions
*===========================================================================*/
 
int initialize_bot(void) {
    if (!rpi_gpio_map_regs(RPI_4_PERIPHERALS)) {
        perror("GPIO mapping failed");
        return FAILURE;
    }

    return SUCCESS;
}
 
int initialize_button(uint32_t button) {
    rpi_gpio_set_select(button, RPI_GPIO_FUNC_IN);
    if (!rpi_gpio_set_pud_bcm2711(button, RPI_GPIO_PUD_UP)) {
        fprintf(stderr, "Pull-up setup failed for button %d\n", button);
        rpi_gpio_unmap_regs();
        return FAILURE;
    }
    return SUCCESS;
}
 
int read_button(uint32_t button) {
    return rpi_gpio_read(button);
}
 
int initialize_led(uint32_t led) {
    rpi_gpio_set_select(led, RPI_GPIO_FUNC_OUT);
    return SUCCESS;
}
 
int set_led(uint32_t led, uint32_t state) {
    if (state == 0)
        rpi_gpio_clear(led);
    else
        rpi_gpio_set(led);
    return SUCCESS;
}
 
/*===========================================================================
* Ultrasonic Sensor Implementation
*===========================================================================*/
 
/* Helper: Calculate elapsed time (microseconds) */
static float calculate_elapsed_time_us(const struct timespec *start, const struct timespec *stop) {
    return (stop->tv_sec - start->tv_sec) * 1e6f +
           (stop->tv_nsec - start->tv_nsec) / 1e3f;
}
 
/* Helper: Wait for a GPIO pin to reach a desired state */
static int wait_for_gpio_state(int gpio_pin, bool desired_state, double timeout_ms, struct timespec *timestamp) {
    struct timespec start, current;
    clock_gettime(CLOCK_REALTIME, &start);
    while (1) {
        bool current_state = (rpi_gpio_read(gpio_pin) != 0);
        if (current_state == desired_state) {
            clock_gettime(CLOCK_REALTIME, timestamp);
            return 0;
        }
        clock_gettime(CLOCK_REALTIME, &current);
        double elapsed_ms = (current.tv_sec - start.tv_sec) * 1000.0 +
                            (current.tv_nsec - start.tv_nsec) / 1e6;
        if (elapsed_ms > timeout_ms)
            return -1;
    }
}
 
/* Helper: Send a 10-µs pulse on a given GPIO pin */
static int send_pulse(uint32_t gpio_pin) {
    rpi_gpio_set(gpio_pin);
    struct timespec pulse_duration = { .tv_sec = 0, .tv_nsec = 10 * 1000 };  // 10 µs
    nanosleep(&pulse_duration, NULL);
    rpi_gpio_clear(gpio_pin);
    return 0;
}
 
int ultrasonic_init(void) {
    // Configure trigger and echo pins.
    rpi_gpio_set_select(GPIO_PULSE_PIN, RPI_GPIO_FUNC_OUT);
    rpi_gpio_set_select(GPIO_INTERRUPT_PIN, RPI_GPIO_FUNC_IN);
    if (!rpi_gpio_set_pud_bcm2711(GPIO_INTERRUPT_PIN, RPI_GPIO_PUD_OFF)) {
        printf("Failed to disable pull resistor on echo pin\n");
        return FAILURE;
    }
    __RPI_GPIO_REGS[RPI_GPIO_REG_GPEDS0] = 0xFFFFFFFF;
    return SUCCESS;
}
 
int ultrasonic_read_distance(float *distance) {
    if (send_pulse(GPIO_PULSE_PIN) != 0) {
        printf("Failed to send trigger pulse\n");
        return FAILURE;
    }
    struct timespec rising_edge_time, falling_edge_time;
    if (wait_for_gpio_state(GPIO_INTERRUPT_PIN, true, 50.0, &rising_edge_time) != 0) {
        printf("Timeout waiting for rising edge\n");
        return FAILURE;
    }
    if (wait_for_gpio_state(GPIO_INTERRUPT_PIN, false, 50.0, &falling_edge_time) != 0) {
        printf("Timeout waiting for falling edge\n");
        return FAILURE;
    }
    float pulse_duration_us = calculate_elapsed_time_us(&rising_edge_time, &falling_edge_time);
    *distance = (pulse_duration_us * 0.0343f) / 2.0f;
    return SUCCESS;
}
 
/*===========================================================================
* SN3218 LED Driver Implementation
*===========================================================================*/
 
// Mapping for 6 RGB LEDs.
static const uint8_t channel_map[SN3218_NUM_CHANNELS] = {
    0,  1,  2,    // LED0
    3,  4,  5,    // LED1
    6,  7,  8,    // LED2
    9,  10, 11,   // LED3
    12, 13, 14,   // LED4
    15, 16, 17    // LED5
};
 
static uint8_t g_brightness[SN3218_NUM_CHANNELS] = {0};
 
static int sn3218_write_byte(uint8_t reg, uint8_t value) {
    int rc = smbus_write_byte_data(I2C_BUS, SN3218_ADDR, reg, value);
    if (rc != I2C_SUCCESS)
        fprintf(stderr, "ERROR: Writing reg 0x%02X, val 0x%02X; rc=%d\n", reg, value, rc);
    return rc;
}
 
static int sn3218_write_block_loop(uint8_t start_reg, const uint8_t *data, uint8_t length) {
    int rc = I2C_SUCCESS;
    for (uint8_t i = 0; i < length; i++) {
        rc = smbus_write_byte_data(I2C_BUS, SN3218_ADDR, start_reg + i, data[i]);
        if (rc != I2C_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to write reg 0x%02X\n", start_reg + i);
            return rc;
        }
        usleep(1000);
    }
    return rc;
}
 
static int sn3218_reset(void) {
    int rc = sn3218_write_byte(REG_RESET, 0xFF);
    usleep(10000);
    return rc;
}
 
static int sn3218_init(void) {
    int rc = sn3218_reset();
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 reset failed.\n");
        return rc;
    }
    rc = sn3218_write_byte(REG_SHUTDOWN, 0x01);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 shutdown disable failed.\n");
        return rc;
    }
    usleep(10000);
    uint8_t enable_data[3] = {0x3F, 0x3F, 0x3F};
    rc = sn3218_write_block_loop(REG_ENABLE_LEDS, enable_data, 3);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 channel enable failed.\n");
        return rc;
    }
    usleep(10000);
    for (int i = 0; i < SN3218_NUM_CHANNELS; i++) {
        g_brightness[i] = 0;
    }
    rc = sn3218_write_block_loop(REG_PWM_BASE, g_brightness, SN3218_NUM_CHANNELS);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 initial brightness set failed.\n");
        return rc;
    }
    rc = sn3218_write_byte(REG_UPDATE, 0x00);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 update latch failed.\n");
        return rc;
    }
    return I2C_SUCCESS;
}
 
static int sn3218_update(void) {
    int rc = sn3218_write_block_loop(REG_PWM_BASE, g_brightness, SN3218_NUM_CHANNELS);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "SN3218 brightness update failed.\n");
        return rc;
    }
    rc = sn3218_write_byte(REG_UPDATE, 0x00);
    return rc;
}
 
static void sn3218_set_channel(uint8_t channel, uint8_t brightness) {
    if (channel < SN3218_NUM_CHANNELS) {
        uint8_t physical_ch = channel_map[channel];
        g_brightness[physical_ch] = brightness;
    }
}
 
static void sn3218_set_rgb(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (led_index < 6) {
        uint8_t base = led_index * 3;
        sn3218_set_channel(base, r);
        sn3218_set_channel(base + 1, g);
        sn3218_set_channel(base + 2, b);
    }
}
 
static void sn3218_test_pattern(void) {
    sn3218_set_rgb(0, 255, 0, 0);
    sn3218_set_rgb(1, 0, 255, 0);
    sn3218_set_rgb(2, 0, 0, 255);
    sn3218_set_rgb(3, 255, 255, 0);
    sn3218_set_rgb(4, 255, 0, 255);
    sn3218_set_rgb(5, 0, 255, 255);
}
 
static void channel_walk_test(void) {
    for (int ch = 0; ch < SN3218_NUM_CHANNELS; ch++) {
        for (int i = 0; i < SN3218_NUM_CHANNELS; i++)
            g_brightness[i] = 0;
        g_brightness[ch] = 255;
        sn3218_write_block_loop(REG_PWM_BASE, g_brightness, SN3218_NUM_CHANNELS);
        sn3218_write_byte(REG_UPDATE, 0x00);
        sleep(1);
    }
    for (int i = 0; i < SN3218_NUM_CHANNELS; i++)
        g_brightness[i] = 0;
    sn3218_write_block_loop(REG_PWM_BASE, g_brightness, SN3218_NUM_CHANNELS);
    sn3218_write_byte(REG_UPDATE, 0x00);
}
 
/* Public LED API */
int trilobot_led_init(void) {
    return (sn3218_init() == I2C_SUCCESS) ? SUCCESS : FAILURE;
}
void trilobot_led_set_channel(uint8_t channel, uint8_t brightness) {
    sn3218_set_channel(channel, brightness);
}
void trilobot_led_set_rgb(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    sn3218_set_rgb(led_index, r, g, b);
}
int trilobot_led_update(void) {
    return (sn3218_update() == I2C_SUCCESS) ? SUCCESS : FAILURE;
}
void trilobot_led_test_pattern(void) {
    sn3218_test_pattern();
}
void trilobot_led_channel_walk_test(void) {
    channel_walk_test();
}

/*-----------------------------------------------------------
* LED Animation & Button Sequence API Implementation
*-----------------------------------------------------------
*/
 
// Internal buffer to hold the last 4 button events.
static char trilobot_button_seq[5] = {0};  // 4 characters + null terminator
 
void trilobot_reset_button_sequence(void) {
    for (int i = 0; i < 5; i++) {
        trilobot_button_seq[i] = '\0';
    }
}
 
void trilobot_record_button(char btn) {
    // Shift the sequence left by one.
    for (int i = 0; i < 4 - 1; i++) {
        trilobot_button_seq[i] = trilobot_button_seq[i+1];
    }
    trilobot_button_seq[3] = btn;
    trilobot_button_seq[4] = '\0';
}
 
bool trilobot_check_special_sequence(void) {
    // For example, if sequence equals "AABB" or "XXYY", return true.
    if ((strcmp(trilobot_button_seq, "AABB") == 0) ||
        (strcmp(trilobot_button_seq, "XXYY") == 0)) {
        return true;
    }
    return false;
}
 
void trilobot_run_rgb_loop(void) {
    const int delay_ms = 150;
    // Run the loop for several cycles.
    for (int cycle = 0; cycle < 20; cycle++) {
        // Red
        for (int i = 0; i < 6; i++) {
            trilobot_led_set_rgb(i, 255, 0, 0);
        }
        trilobot_led_update();
        usleep(delay_ms * 1000);
        
        // Green
        for (int i = 0; i < 6; i++) {
            trilobot_led_set_rgb(i, 0, 255, 0);
        }
        trilobot_led_update();
        usleep(delay_ms * 1000);
        
        // Blue
        for (int i = 0; i < 6; i++) {
            trilobot_led_set_rgb(i, 0, 0, 255);
        }
        trilobot_led_update();
        usleep(delay_ms * 1000);
        
        // Off
        for (int i = 0; i < 6; i++) {
            trilobot_led_set_rgb(i, 0, 0, 0);
        }
        trilobot_led_update();
        usleep(delay_ms * 1000);
    }
}

const char* trilobot_get_button_sequence(void) {
    return trilobot_button_seq;
}
/*===========================================================================
* Motor Driver (SW PWM) Implementation
*===========================================================================*/
 
static int sw_pwm_pin;                     // Pin used for SW PWM.
static unsigned int sw_pwm_period_us;      // PWM period in microseconds.
static volatile float sw_pwm_duty = 0.0f;    // Duty cycle (0.0 - 100.0).
static volatile int sw_pwm_running = 0;      // Flag for PWM loop.
static pthread_t sw_pwm_thread;
static pthread_mutex_t sw_pwm_mutex = PTHREAD_MUTEX_INITIALIZER;
 
static void* sw_pwm_loop(void *arg) {
    (void)arg;
    while (sw_pwm_running) {
        float duty;
        pthread_mutex_lock(&sw_pwm_mutex);
        duty = sw_pwm_duty;
        pthread_mutex_unlock(&sw_pwm_mutex);
        
        if (duty <= 0.0f) {
            rpi_gpio_output(sw_pwm_pin, GPIO_LOW);
            usleep(sw_pwm_period_us);
        } else if (duty >= 100.0f) {
            rpi_gpio_output(sw_pwm_pin, GPIO_HIGH);
            usleep(sw_pwm_period_us);
        } else {
            unsigned int on_time = (unsigned int)((duty / 100.0f) * sw_pwm_period_us);
            unsigned int off_time = sw_pwm_period_us - on_time;
            rpi_gpio_output(sw_pwm_pin, GPIO_HIGH);
            usleep(on_time);
            rpi_gpio_output(sw_pwm_pin, GPIO_LOW);
            usleep(off_time);
        }
    }
    return NULL;
}
 
static int sw_pwm_init(int pin, unsigned int frequency) {
    int rc;
    sw_pwm_pin = pin;
    sw_pwm_period_us = 1000000 / frequency;
    
    rc = rpi_gpio_setup(sw_pwm_pin, GPIO_OUT);
    if (rc != GPIO_SUCCESS) {
        printf("ERROR: rpi_gpio_setup() failed for SW PWM pin %d, rc=%d\n", sw_pwm_pin, rc);
        return rc;
    }
    
    pthread_mutex_lock(&sw_pwm_mutex);
    sw_pwm_duty = 0.0f;
    pthread_mutex_unlock(&sw_pwm_mutex);
    
    sw_pwm_running = 1;
    if (pthread_create(&sw_pwm_thread, NULL, sw_pwm_loop, NULL) != 0) {
        perror("pthread_create");
        sw_pwm_running = 0;
        return -1;
    }
    return GPIO_SUCCESS;
}
 
static void sw_pwm_cleanup(void) {
    if (sw_pwm_running) {
        sw_pwm_running = 0;
        pthread_join(sw_pwm_thread, NULL);
    }
}
 
static int setup_pwm_pin(int gpio_pin) {
    return sw_pwm_init(gpio_pin, PWM_FREQ);
}
 
/* Public Motor API */
int motor_init(void) {
    int rc;
    printf("[motor_init] Initializing motor driver...\n");
    rc = setup_pwm_pin(MOTOR_EN_PIN);
    if (rc != GPIO_SUCCESS)
        return rc;
    if (rpi_gpio_setup(MOTOR_RP_PIN, GPIO_OUT) != GPIO_SUCCESS ||
        rpi_gpio_setup(MOTOR_RN_PIN, GPIO_OUT) != GPIO_SUCCESS ||
        rpi_gpio_setup(MOTOR_LP_PIN, GPIO_OUT) != GPIO_SUCCESS ||
        rpi_gpio_setup(MOTOR_LN_PIN, GPIO_OUT) != GPIO_SUCCESS) {
        printf("ERROR: Failed to setup motor direction pins.\n");
        return FAILURE;
    }
    rpi_gpio_output(MOTOR_RP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_RN_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_LP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_LN_PIN, GPIO_LOW);
    printf("[motor_init] Motor driver initialization complete.\n");
    return GPIO_SUCCESS;
}
 
int motor_set_speed(float speed_percent) {
    pthread_mutex_lock(&sw_pwm_mutex);
    sw_pwm_duty = speed_percent;
    pthread_mutex_unlock(&sw_pwm_mutex);
    return GPIO_SUCCESS;
}
 
int motor_disable(void) {
    return motor_set_speed(0.0f);
}
 
int motor_right_forward(void) {
    rpi_gpio_output(MOTOR_RP_PIN, GPIO_HIGH);
    rpi_gpio_output(MOTOR_RN_PIN, GPIO_LOW);
    return GPIO_SUCCESS;
}
 
int motor_right_reverse(void) {
    rpi_gpio_output(MOTOR_RP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_RN_PIN, GPIO_HIGH);
    return GPIO_SUCCESS;
}
 
int motor_left_forward(void) {
    rpi_gpio_output(MOTOR_LP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_LN_PIN, GPIO_HIGH);
    return GPIO_SUCCESS;
}
 
int motor_left_reverse(void) {
    rpi_gpio_output(MOTOR_LP_PIN, GPIO_HIGH);
    rpi_gpio_output(MOTOR_LN_PIN, GPIO_LOW);
    return GPIO_SUCCESS;
}
 
int motor_stop(void) {
    rpi_gpio_output(MOTOR_RP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_RN_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_LP_PIN, GPIO_LOW);
    rpi_gpio_output(MOTOR_LN_PIN, GPIO_LOW);
    motor_set_speed(0.0f);
    return GPIO_SUCCESS;
}
 
int motor_turn_left(void) {
    motor_left_reverse();
    motor_right_forward();
    return GPIO_SUCCESS;
}
 
int motor_turn_right(void) {
    motor_left_forward();
    motor_right_reverse();
    return GPIO_SUCCESS;
}
 
void motor_sigint_handler(int sig) {
    (void)sig;
    printf("\nSIGINT received, disabling motor driver...\n");
    motor_disable();
    sw_pwm_cleanup();
    exit(EXIT_SUCCESS);
}
 
/*===========================================================================
* IMU (ICM‑20948) Implementation
*===========================================================================*/
 

 
static int icm20948_select_bank(uint8_t bank) {
    return smbus_write_byte_data(I2C_BUS, ICM20948_ADDR, REG_BANK_SEL, bank);
}
 
static int icm20948_write_reg(uint8_t reg, uint8_t data) {
    return smbus_write_byte_data(I2C_BUS, ICM20948_ADDR, reg, data);
}
 
static int icm20948_read_reg(uint8_t reg, uint8_t *data) {
    return smbus_read_byte_data(I2C_BUS, ICM20948_ADDR, reg, data);
}
 
static int icm20948_init(void) {
    int rc;
    uint8_t whoami = 0;
    rc = icm20948_select_bank(BANK_0);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not switch to BANK_0\n");
        return rc;
    }
    rc = icm20948_read_reg(WHO_AM_I, &whoami);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not read WHO_AM_I\n");
        return rc;
    }
    printf("ICM-20948 WHO_AM_I = 0x%02X (expected 0xEA)\n", whoami);
    rc = icm20948_write_reg(PWR_MGMT_1, 0x01);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not write PWR_MGMT_1\n");
        return rc;
    }
    rc = icm20948_write_reg(PWR_MGMT_2, 0x00);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not write PWR_MGMT_2\n");
        return rc;
    }
    usleep(10000);
    rc = icm20948_select_bank(BANK_2);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not switch to BANK_2\n");
        return rc;
    }
    rc = icm20948_write_reg(ACCEL_CONFIG, 0x01);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not write ACCEL_CONFIG\n");
        return rc;
    }
    rc = icm20948_write_reg(GYRO_CONFIG_1, 0x01);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not write GYRO_CONFIG_1\n");
        return rc;
    }
    rc = icm20948_select_bank(BANK_0);
    if (rc != I2C_SUCCESS) {
        fprintf(stderr, "ERROR: Could not switch back to BANK_0\n");
        return rc;
    }
    printf("ICM-20948 initialization complete.\n");
    return I2C_SUCCESS;
}
 
static int icm20948_read_sensors(int16_t *ax, int16_t *ay, int16_t *az,
                                 int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t raw[12];
    int rc = smbus_read_block_data(I2C_BUS, ICM20948_ADDR, ACCEL_XOUT_H, raw, 12);
    if (rc != I2C_SUCCESS) return rc;
    *ax = (int16_t)((raw[0] << 8) | raw[1]);
    *ay = (int16_t)((raw[2] << 8) | raw[3]);
    *az = (int16_t)((raw[4] << 8) | raw[5]);
    *gx = (int16_t)((raw[6] << 8) | raw[7]);
    *gy = (int16_t)((raw[8] << 8) | raw[9]);
    *gz = (int16_t)((raw[10] << 8) | raw[11]);
    return I2C_SUCCESS;
}
 
static int calibrate_imu(int num_samples,
                         double *accel_offset_x, double *accel_offset_y, double *accel_offset_z,
                         double *gyro_offset_x, double *gyro_offset_y, double *gyro_offset_z) {
    int rc;
    int16_t ax, ay, az, gx, gy, gz;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    printf("Calibrating IMU... keep sensor still.\n");
    for (int i = 0; i < num_samples; i++) {
        rc = icm20948_read_sensors(&ax, &ay, &az, &gx, &gy, &gz);
        if (rc != I2C_SUCCESS) {
            fprintf(stderr, "Calibration error at sample %d\n", i);
            return rc;
        }
        sum_ax += ax / ACCEL_SENSITIVITY;
        sum_ay += ay / ACCEL_SENSITIVITY;
        sum_az += az / ACCEL_SENSITIVITY;
        sum_gx += gx / GYRO_SENSITIVITY;
        sum_gy += gy / GYRO_SENSITIVITY;
        sum_gz += gz / GYRO_SENSITIVITY;
        usleep(5000);
    }
    *accel_offset_x = sum_ax / num_samples;
    *accel_offset_y = sum_ay / num_samples;
    *accel_offset_z = sum_az / num_samples;
    *gyro_offset_x  = sum_gx / num_samples;
    *gyro_offset_y  = sum_gy / num_samples;
    *gyro_offset_z  = sum_gz / num_samples;
    printf("Calibration complete.\n");
    return I2C_SUCCESS;
}
 
/* Public IMU API */
int imu_init(void) {
    return (icm20948_init() == I2C_SUCCESS) ? SUCCESS : FAILURE;
}
int imu_calibrate(int num_samples,
                  double *accel_offset_x, double *accel_offset_y, double *accel_offset_z,
                  double *gyro_offset_x, double *gyro_offset_y, double *gyro_offset_z) {
    return calibrate_imu(num_samples,
                         accel_offset_x, accel_offset_y, accel_offset_z,
                         gyro_offset_x, gyro_offset_y, gyro_offset_z);
}
int imu_read_sensors(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz) {
    return icm20948_read_sensors(ax, ay, az, gx, gy, gz);
}
 
/*===========================================================================
* End of trilobot.c
*===========================================================================*/
