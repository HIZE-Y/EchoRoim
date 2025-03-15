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
* @file rgb_led.c
**************************************************************************/
#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
 
/* Signal handler to clean up the LED driver before exiting */
static void cleanup_leds(int sig) {
    (void)sig;  // Unused parameter
    printf("\nSIGINT received: turning off all LEDs and exiting...\n");
    // Turn off all 6 RGB LEDs.
    for (int i = 0; i < 6; i++) {
        trilobot_led_set_rgb(i, 0, 0, 0);
    }
    // Update the LED driver to latch the changes.
    trilobot_led_update();
    exit(EXIT_SUCCESS);
}
 
int main(void) {
    // Install the signal handler for SIGINT.
    signal(SIGINT, cleanup_leds);
    
    // Initialize the bot (maps GPIO registers, etc.).
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Initialize the SN3218 LED driver.
    if (trilobot_led_init() != SUCCESS) {
        fprintf(stderr, "LED driver initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Display a fixed test pattern for 3 seconds.
    printf("Displaying fixed test pattern...\n");
    trilobot_led_test_pattern();
    if (trilobot_led_update() != SUCCESS) {
        fprintf(stderr, "LED update failed\n");
        return EXIT_FAILURE;
    }
    sleep(3);
    
    // Start continuous RGB loop using the provided function.
    printf("Starting continuous RGB loop...\n");
    while (1) {
        trilobot_run_rgb_loop();
    }
    
    return EXIT_SUCCESS;
}
