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
* @file ultrasonic.c
**************************************************************************/
#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
 
int main(void) {
    float distance;
 
    // Initialize the bot (this maps the GPIO registers, etc.).
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
 
    // Initialize the ultrasonic sensor.
    if (ultrasonic_init() != SUCCESS) {
        fprintf(stderr, "Ultrasonic sensor initialization failed\n");
        return EXIT_FAILURE;
    }
 
    // Allow the sensor to settle.
    sleep(2);
 
    // Continuously measure and print the distance.
    while (1) {
        if (ultrasonic_read_distance(&distance) == SUCCESS) {
            printf("Distance: %.2f cm\n", distance);
        } else {
            printf("Error reading distance\n");
        }
        usleep(100000);  // Delay 100 ms between measurements.
    }
 
    return EXIT_SUCCESS;
}
