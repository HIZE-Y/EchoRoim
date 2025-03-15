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
* @file motor_sw_pwm.c
**************************************************************************/
#include "trilobot.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
 
int main(void) {
    // Initialize the bot.
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Install SIGINT handler for safe shutdown.
    signal(SIGINT, motor_sigint_handler);
    
    // Initialize the motor driver.
    if (motor_init() != GPIO_SUCCESS) {
        fprintf(stderr, "Motor initialization failed\n");
        return EXIT_FAILURE;
    }
    
    sleep(10);

    // Drive forward.
    printf("Driving forward...\n");
    motor_right_forward();
    motor_left_forward();
    motor_set_speed(50.0f);  // 50% speed
    sleep(2);
    
    // Drive in reverse.
    printf("Driving reverse...\n");
    motor_right_reverse();
    motor_left_reverse();
    motor_set_speed(50.0f);
    sleep(2);
    
    // Pivot turn left.
    printf("Pivot turning left...\n");
    motor_turn_left();
    motor_set_speed(30.0f);
    sleep(2);
    
    // Pivot turn right.
    printf("Pivot turning right...\n");
    motor_turn_right();
    motor_set_speed(30.0f);
    sleep(2);
    
    // Stop the motors.
    printf("Stopping motors...\n");
    motor_stop();
    sleep(1);
     
    motor_right_forward();
    motor_left_forward();
    
    // Ramp speed from 0% to 100% and back.
    printf("Ramping speed up...\n");
    for (int s = 0; s <= 100; s += 10) {
        printf("Speed: %d%%\n", s);
        motor_set_speed((float)s);
        sleep(1);
    }
    printf("Ramping speed down...\n");
    for (int s = 100; s >= 0; s -= 10) {
        printf("Speed: %d%%\n", s);
        motor_set_speed((float)s);
        sleep(1);
    }
    
    motor_stop();
    return EXIT_SUCCESS;
}
