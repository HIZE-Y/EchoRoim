#include "trilobot.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>

// Global flag for program state
volatile int running = 1;

// Global variables for sensor data sharing between threads
volatile float current_distance = 1000.0f;  // Start with a large value
pthread_mutex_t distance_mutex = PTHREAD_MUTEX_INITIALIZER;

// Constants for obstacle avoidance
#define OBSTACLE_THRESHOLD 20.0f     // Distance in cm to trigger avoidance
#define BACKUP_DISTANCE   20.0f     // Distance to back up in cm
#define TURN_ANGLE       100.0f      // Angle to turn in degrees
#define SAFE_SPEED       60.0f      // Normal forward speed
#define BACKUP_SPEED     40.0f      // Speed when backing up
#define TURN_SPEED       65.0f      // Speed during turns

// Robot physical characteristics (these are approximate values)
#define WHEEL_DIAMETER   6.5f       // Wheel diameter in cm
#define WHEEL_BASE      12.0f       // Distance between wheels in cm
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define ROBOT_CIRCUMFERENCE (WHEEL_BASE * M_PI)

// Function to calculate time needed to move a certain distance at a given speed
// speed is in percentage (0-100), distance in cm, returns microseconds
uint32_t calculate_move_time(float speed, float distance) {
    // At 100% speed, assume we move about 30cm per second (adjust this based on actual robot performance)
    float max_speed = 30.0f;  // cm per second at 100% speed
    float actual_speed = (speed / 100.0f) * max_speed;  // cm per second at given speed
    float time_seconds = distance / actual_speed;
    return (uint32_t)(time_seconds * 1000000);  // Convert to microseconds
}

// Function to calculate time needed to turn a certain angle at a given speed
// speed is in percentage (0-100), angle in degrees, returns microseconds
uint32_t calculate_turn_time(float speed, float angle) {
    // Calculate the arc length for the turn
    float arc_length = (ROBOT_CIRCUMFERENCE * angle) / 360.0f;
    
    // At 100% speed, assume we can complete a 360-degree turn in about 2 seconds (adjust based on actual robot)
    float max_angular_speed = 180.0f;  // degrees per second at 100% speed
    float actual_angular_speed = (speed / 100.0f) * max_angular_speed;
    float time_seconds = angle / actual_angular_speed;
    return (uint32_t)(time_seconds * 1000000);  // Convert to microseconds
}

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

// Function to perform the avoidance maneuver
void perform_avoidance_maneuver(void) {
    // First back up
    printf("Backing up %.1f cm...\n", BACKUP_DISTANCE);
    motor_right_reverse();
    motor_left_reverse();
    motor_set_speed(BACKUP_SPEED);
    set_status_led(255, 165, 0);  // Orange - backing up
    usleep(calculate_move_time(BACKUP_SPEED, BACKUP_DISTANCE));
    
    // Then turn right
    printf("Turning right %.1f degrees...\n", TURN_ANGLE);
    motor_turn_right();
    motor_set_speed(TURN_SPEED);
    set_status_led(255, 0, 0);  // Red - turning
    usleep(calculate_turn_time(TURN_SPEED, TURN_ANGLE));
}

int main(void) {
    pthread_t sensor_thread;
    float distance;
    
    // Initialize the bot
    if (initialize_bot() != SUCCESS) {
        fprintf(stderr, "Bot initialization failed\n");
        return EXIT_FAILURE;
    }
    
    // Install signal handler
    signal(SIGINT, sigint_handler);
    
    // Initialize motor driver
    if (motor_init() != GPIO_SUCCESS) {
        fprintf(stderr, "Motor initialization failed\n");
        return EXIT_FAILURE;
    }

    // Initialize LED driver
    if (trilobot_led_init() != SUCCESS) {
        fprintf(stderr, "LED initialization failed\n");
        return EXIT_FAILURE;
    }

    // Start ultrasonic sensor thread
    if (pthread_create(&sensor_thread, NULL, ultrasonic_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create ultrasonic sensor thread\n");
        return EXIT_FAILURE;
    }

    printf("Starting obstacle avoidance...\n");
    
    // Main control loop
    while (running) {
        // Get current distance reading
        pthread_mutex_lock(&distance_mutex);
        distance = current_distance;
        pthread_mutex_unlock(&distance_mutex);
        
        if (distance < OBSTACLE_THRESHOLD) {
            // Obstacle detected - perform avoidance maneuver
            printf("Obstacle detected at %.2f cm! Avoiding...\n", distance);
            motor_stop();
            
            // Keep trying to avoid until we find a clear path
            while (distance < OBSTACLE_THRESHOLD && running) {
                perform_avoidance_maneuver();
                
                // Check distance again
                pthread_mutex_lock(&distance_mutex);
                distance = current_distance;
                pthread_mutex_unlock(&distance_mutex);
                
                // Brief pause to let sensor readings stabilize
                usleep(100000);  // 100ms pause
            }
            
        } else {
            // No obstacle - move forward
            motor_right_forward();
            motor_left_forward();
            motor_set_speed(SAFE_SPEED);
            set_status_led(0, 255, 0);  // Green - path is clear
        }
        
        usleep(50000);  // 50ms control loop
    }

    // Wait for ultrasonic thread to finish
    pthread_join(sensor_thread, NULL);
    return EXIT_SUCCESS;
} 