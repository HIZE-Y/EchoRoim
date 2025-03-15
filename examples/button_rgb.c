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
* @file button_rgb.c
**************************************************************************/
#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
 
/* Signal handler to turn off all LEDs and exit gracefully */
static void cleanup_leds(int sig) {
    (void)sig;  // Unused parameter
    printf("\nSIGINT received, turning off LEDs and exiting...\n");
    for (int i = 0; i < 6; i++) {
        trilobot_led_set_rgb(i, 0, 0, 0);
    }
    trilobot_led_update();
    exit(EXIT_SUCCESS);
}
 
int main(void) {
    // Install SIGINT handler.
    signal(SIGINT, cleanup_leds);
    
    // Initialize the bot.
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Initialize the LED driver.
    if (trilobot_led_init() != SUCCESS) {
        fprintf(stderr, "LED driver initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Initialize buttons A, B, X, Y.
    if (initialize_button(Button_A) != SUCCESS ||
        initialize_button(Button_B) != SUCCESS ||
        initialize_button(Button_X) != SUCCESS ||
        initialize_button(Button_Y) != SUCCESS) {
        fprintf(stderr, "Button initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Reset the internal button sequence buffer.
    trilobot_reset_button_sequence();
    
    printf("Waiting for button input...\n");
    
    while (1) {
        // For each button, check if it is pressed (0 indicates pressed).
        if (read_button(Button_A) == 0) {
            trilobot_record_button('A');
            // Set LEDs to red.
            for (int i = 0; i < 6; i++) {
                trilobot_led_set_rgb(i, 255, 0, 0);
            }
            trilobot_led_update();
            usleep(300000);  // Debounce.
        }
        if (read_button(Button_B) == 0) {
            trilobot_record_button('B');
            // Set LEDs to green.
            for (int i = 0; i < 6; i++) {
                trilobot_led_set_rgb(i, 0, 255, 0);
            }
            trilobot_led_update();
            usleep(300000);
        }
        if (read_button(Button_X) == 0) {
            trilobot_record_button('X');
            // Set LEDs to blue.
            for (int i = 0; i < 6; i++) {
                trilobot_led_set_rgb(i, 0, 0, 255);
            }
            trilobot_led_update();
            usleep(300000);
        }
        if (read_button(Button_Y) == 0) {
            trilobot_record_button('Y');
            // Set LEDs to white.
            for (int i = 0; i < 6; i++) {
                trilobot_led_set_rgb(i, 255, 255, 255);
            }
            trilobot_led_update();
            usleep(300000);
        }
        
        // If a special sequence (e.g., "AABB" or "XXYY") is detected, run the RGB loop.
        if (trilobot_check_special_sequence()) {
            printf("Special sequence detected!(%s) Running RGB loop animation...\n", trilobot_get_button_sequence());
            trilobot_run_rgb_loop();
            trilobot_reset_button_sequence();
        }
        
        usleep(10000);  // Poll every 10ms.
    }
    
    return EXIT_SUCCESS;
}
