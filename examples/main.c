#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>
#include <time.h>   // For random seed

// Global flag for program state
volatile int running = 1;

// Global variables for sensor data sharing between threads
volatile float current_distance = 1000.0f;  // Start with a large value
pthread_mutex_t distance_mutex = PTHREAD_MUTEX_INITIALIZER;

// Constants for movement
#define OBSTACLE_THRESHOLD 20.0f     // Distance in cm to trigger avoidance
#define BACKUP_DISTANCE   20.0f     // Distance to back up in cm
#define TURN_ANGLE       90.0f      // Turn angle in degrees
#define SAFE_SPEED       260.0f     // Normal forward speed
#define BACKUP_SPEED     240.0f     // Speed when backing up
#define TURN_SPEED       265.0f     // Speed during turns
#define MAX_SAME_TURNS   4          // Maximum number of same-direction turns before forcing a change

// Function to handle Ctrl+C gracefully
void sigint_handler(int sig) {
    (void)sig;
    printf("\nSIGINT received, stopping motors and LEDs...\n");
    running = 0;  // Signal threads to stop
    motor_stop();
    motor_disable();
    // Turn off all LEDs
    for (int i = 0; i < 6; i++) {
        trilobot_led_set_rgb(i, 0, 0, 0);
    }
    trilobot_led_update();
    exit(EXIT_SUCCESS);
}

// Thread function for ultrasonic sensor
void* ultrasonic_thread(void* arg) {
    (void)arg;  // Unused parameter
    float distance;
    
    // Initialize ultrasonic sensor
    if (ultrasonic_init() != SUCCESS) {
        fprintf(stderr, "Ultrasonic sensor initialization failed\n");
        return NULL;
    }
    
    // Allow sensor to settle
    sleep(1);
    
    // Continuously measure distance
    while (running) {
        if (ultrasonic_read_distance(&distance) == SUCCESS) {
            pthread_mutex_lock(&distance_mutex);
            current_distance = distance;
            pthread_mutex_unlock(&distance_mutex);
            printf("Distance: %.2f cm\n", distance);
        } else {
            printf("Error reading distance\n");
        }
        usleep(100000);  // 100ms between readings
    }
    
    return NULL;
}

// Function to set LED color based on state
void set_status_led(int red, int green, int blue) {
    for (int i = 0; i < 6; i++) {
        trilobot_led_set_rgb(i, red, green, blue);
    }
    trilobot_led_update();
}

// Function to calculate time needed to move a certain distance at a given speed
uint32_t calculate_move_time(float speed, float distance) {
    float max_speed = 30.0f;  // cm per second at 100% speed
    float actual_speed = (speed / 100.0f) * max_speed;
    float time_seconds = distance / actual_speed;
    return (uint32_t)(time_seconds * 1000000);
}

// Function to calculate time needed to turn a certain angle at a given speed
uint32_t calculate_turn_time(float speed, float angle) {
    float max_angular_speed = 180.0f;  // degrees per second at 100% speed
    float actual_angular_speed = (speed / 100.0f) * max_angular_speed;
    float time_seconds = angle / actual_angular_speed;
    return (uint32_t)(time_seconds * 1000000);
}

// Function to move backward a specific distance
void move_backward(float distance) {
    printf("Moving backward %.1f cm...\n", distance);
    motor_right_reverse();
    motor_left_reverse();
    motor_set_speed(BACKUP_SPEED);
    set_status_led(255, 165, 0);  // Orange - backing up
    usleep(calculate_move_time(BACKUP_SPEED, distance));
}

// Function to turn with direction choice
void turn(int turn_left) {
    printf("Turning %s %.1f degrees...\n", turn_left ? "left" : "right", TURN_ANGLE);
    if (turn_left) {
        motor_turn_left();
    } else {
        motor_turn_right();
    }
    motor_set_speed(TURN_SPEED);
    set_status_led(255, 0, 0);  // Red - turning
    usleep(calculate_turn_time(TURN_SPEED, TURN_ANGLE));
}

int main(void) {
    pthread_t sensor_thread;
    
    // Initialize everything
    srand(time(NULL));  // Initialize random number generator
    
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    signal(SIGINT, sigint_handler);
    
    if (motor_init() != GPIO_SUCCESS) {
        fprintf(stderr, "Motor initialization failed\n");
        return EXIT_FAILURE;
    }

    if (trilobot_led_init() != SUCCESS) {
        fprintf(stderr, "LED initialization failed\n");
        return EXIT_FAILURE;
    }

    if (pthread_create(&sensor_thread, NULL, ultrasonic_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create ultrasonic sensor thread\n");
        return EXIT_FAILURE;
    }

    printf("Starting continuous movement with 90-degree turns...\n");
    
    // Start moving forward continuously
    motor_right_forward();
    motor_left_forward();
    motor_set_speed(SAFE_SPEED);
    set_status_led(0, 255, 0);  // Green - moving forward
    
    int turn_left = 1;  // Start with left turns
    int same_turn_count = 0;  // Count of consecutive same-direction turns
    
    // Main control loop
    while (running) {
        // Get current distance reading
        pthread_mutex_lock(&distance_mutex);
        float distance = current_distance;
        pthread_mutex_unlock(&distance_mutex);
        
        // Check for obstacles
        if (distance < OBSTACLE_THRESHOLD) {
            printf("Obstacle detected at %.2f cm!\n", distance);
            motor_stop();
            
            // Back up
            move_backward(BACKUP_DISTANCE);
            motor_stop();
            
            // Only change direction if we've made too many same-direction turns
            // or if we might be stuck (10% chance)
            if (same_turn_count >= MAX_SAME_TURNS || (rand() % 10 == 0)) {
                turn_left = !turn_left;  // Change direction
                same_turn_count = 0;
                printf("Changing turn direction to %s\n", turn_left ? "left" : "right");
            } else {
                same_turn_count++;
            }
            
            // Perform turn
            turn(turn_left);
            motor_stop();
            
            // Resume forward movement
            motor_right_forward();
            motor_left_forward();
            motor_set_speed(SAFE_SPEED);
            set_status_led(0, 255, 0);  // Green - moving forward
            
            usleep(100000);  // 100ms pause
        }
        
        usleep(50000);  // 50ms main loop
    }

    pthread_join(sensor_thread, NULL);
    return EXIT_SUCCESS;
} 