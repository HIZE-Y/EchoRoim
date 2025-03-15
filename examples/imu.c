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
* @file imu.c
**************************************************************************/
#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
 

 
// Helper function: Exponential moving average filter.
static double filter(double prev, double new_val, double alpha) {
    return alpha * new_val + (1.0 - alpha) * prev;
}
 
int main(void) {
    int rc;
    int16_t ax, ay, az, gx, gy, gz;
    double elapsed_us;
    struct timespec t_start, t_end;
    
    // Offsets determined during calibration (in g for accelerometer, °/s for gyro)
    double accel_offset_x, accel_offset_y, accel_offset_z;
    double gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    // Variables for filtered sensor data.
    double filt_ax = 0, filt_ay = 0, filt_az = 0;
    double filt_gx = 0, filt_gy = 0, filt_gz = 0;
    int first_reading = 1;
    
    printf("=== ICM‑20948 IMU Test Demo ===\n");
    
    // Initialize the bot (maps GPIO registers, etc.).
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Initialize the IMU.
    if (imu_init() != SUCCESS) {
        fprintf(stderr, "IMU initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Calibrate the IMU.
    printf("Calibrating IMU... please keep the sensor completely still.\n");
    rc = imu_calibrate(CALIBRATION_SAMPLES,
                       &accel_offset_x, &accel_offset_y, &accel_offset_z,
                       &gyro_offset_x, &gyro_offset_y, &gyro_offset_z);
    if (rc != SUCCESS) {
        fprintf(stderr, "IMU calibration failed\n");
        return EXIT_FAILURE;
    }
    
    // Main loop: read sensor data, subtract calibration offsets, filter data,
    // and measure the time taken for each sensor read.
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &t_start);
        rc = imu_read_sensors(&ax, &ay, &az, &gx, &gy, &gz);
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        
        if (rc != SUCCESS) {
            fprintf(stderr, "ERROR: Could not read sensor data\n");
        } else {
            // Calculate elapsed time (in microseconds) and convert to milliseconds.
            elapsed_us = (t_end.tv_sec - t_start.tv_sec) * 1e6 +
                         (t_end.tv_nsec - t_start.tv_nsec) / 1000.0;
            double elapsed_ms = elapsed_us / 1000.0;
            
            // Convert raw readings to physical units (accelerometer in g, then to m/s², gyro in °/s)
            double curr_ax = (ax / ACCEL_SENSITIVITY) - accel_offset_x;
            double curr_ay = (ay / ACCEL_SENSITIVITY) - accel_offset_y;
            double curr_az = (az / ACCEL_SENSITIVITY) - accel_offset_z;
            double curr_gx = (gx / GYRO_SENSITIVITY) - gyro_offset_x;
            double curr_gy = (gy / GYRO_SENSITIVITY) - gyro_offset_y;
            double curr_gz = (gz / GYRO_SENSITIVITY) - gyro_offset_z;
            
            // On first reading, initialize filtered values.
            if (first_reading) {
                filt_ax = curr_ax;
                filt_ay = curr_ay;
                filt_az = curr_az;
                filt_gx = curr_gx;
                filt_gy = curr_gy;
                filt_gz = curr_gz;
                first_reading = 0;
            } else {
                // Apply exponential moving average filter.
                filt_ax = filter(filt_ax, curr_ax, ALPHA);
                filt_ay = filter(filt_ay, curr_ay, ALPHA);
                filt_az = filter(filt_az, curr_az, ALPHA);
                filt_gx = filter(filt_gx, curr_gx, ALPHA);
                filt_gy = filter(filt_gy, curr_gy, ALPHA);
                filt_gz = filter(filt_gz, curr_gz, ALPHA);
            }
            
            // Print the time taken and filtered sensor data.
            printf("Time: %5.2f ms | ACCEL: X=%7.3f m/s², Y=%7.3f m/s², Z=%7.3f m/s² | "
                   "GYRO: X=%6.2f dps, Y=%6.2f dps, Z=%6.2f dps\n",
                   elapsed_ms,
                   filt_ax * 9.80665, filt_ay * 9.80665, filt_az * 9.80665,
                   filt_gx, filt_gy, filt_gz);
        }
        
        usleep(LOOP_DELAY_US);
    }
    
    return EXIT_SUCCESS;
}
