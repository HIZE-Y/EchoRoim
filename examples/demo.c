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

// Constants for movement
#define OBSTACLE_THRESHOLD 20.0f     // Distance in cm to trigger avoidance
#define SAFE_SPEED       260.0f     // Normal forward speed
#define TURN_SPEED       200.0f     // Reduced speed for more precise turns
#define PAUSE_TIME       500000     // 500ms pause between moves
#define FINAL_FORWARD_TIME 3000000  // 3 seconds for final forward movement

// Constants for turn calculations
#define WHEEL_BASE_WIDTH 15.0f      // Distance between wheels in cm
#define WHEEL_DIAMETER   6.5f       // Wheel diameter in cm
#define PI              3.14159f
#define MAX_ANGULAR_SPEED 60.0f    // Maximum angular speed in degrees per second at 100% power

// Function to calculate turn time in microseconds
uint32_t calculate_turn_time(float angle_degrees) {
    // Calculate actual angular speed at current power level
    float actual_angular_speed = (TURN_SPEED / 100.0f) * MAX_ANGULAR_SPEED;
    
    // Calculate time needed for turn
    float time_seconds = fabs(angle_degrees) / actual_angular_speed;
    
    // Convert to microseconds
    return (uint32_t)(time_seconds * 1000000.0f);
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
    (void)arg;
    float distance;
    
    if (ultrasonic_init() != SUCCESS) {
        fprintf(stderr, "Ultrasonic sensor initialization failed\n");
        return NULL;
    }
    
    sleep(1);
    
    while (running) {
        if (ultrasonic_read_distance(&distance) == SUCCESS) {
            pthread_mutex_lock(&distance_mutex);
            current_distance = distance;
            pthread_mutex_unlock(&distance_mutex);
            printf("Distance: %.2f cm\n", distance);
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

// Function to perform a turn based on calculated time
void turn_by_angle(float angle_degrees, int turn_left) {
    uint32_t turn_time = calculate_turn_time(angle_degrees);
    
    printf("Turning %s %.1f degrees...\n", 
           turn_left ? "left" : "right", angle_degrees);
    set_status_led(255, 0, 0);  // Red - turning
    
    if (turn_left) {
        motor_turn_left();
    } else {
        motor_turn_right();
    }
    
    motor_set_speed(TURN_SPEED);
    usleep(turn_time);
    motor_stop();
}

// Function to turn left or right 90 degrees
void turn_90(int turn_left) {
    turn_by_angle(90.0f, turn_left);
}

// Function to turn 180 degrees
void turn_180(void) {
    turn_by_angle(90.0f, 1);  // Use left turn for 180
}

// Function to move forward until obstacle
void move_until_obstacle(int with_pause) {
    printf("Moving forward until obstacle...\n");
    motor_right_forward();
    motor_left_forward();
    motor_set_speed(SAFE_SPEED);
    set_status_led(0, 255, 0);  // Green - moving forward
    
    while (running) {
        pthread_mutex_lock(&distance_mutex);
        float distance = current_distance;
        pthread_mutex_unlock(&distance_mutex);
        
        if (distance < OBSTACLE_THRESHOLD) {
            printf("Obstacle detected at %.2f cm\n", distance);
            motor_stop();
            if (with_pause) {
                usleep(PAUSE_TIME);
            }
            break;
        }
        usleep(50000);  // 50ms loop
    }
}

// Function to move forward for specific time
void move_forward_timed(uint32_t time_us) {
    printf("Moving forward for %.1f seconds...\n", time_us / 1000000.0f);
    motor_right_forward();
    motor_left_forward();
    motor_set_speed(SAFE_SPEED);
    set_status_led(0, 255, 0);  // Green - moving forward
    usleep(time_us);
    motor_stop();
}

int main(void) {
    pthread_t sensor_thread;
    
    // Initialize everything
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

    // Create ultrasonic sensor thread
    if (pthread_create(&sensor_thread, NULL, ultrasonic_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create ultrasonic sensor thread\n");
        return EXIT_FAILURE;
    }

    printf("Starting demo sequence...\n");
    sleep(1);  // Brief pause before starting
    
    // Forward sequence with pauses
    printf("\n=== Forward Sequence (with pauses) ===\n");
    move_until_obstacle(1);          // Forward until obstacle
    turn_90(1);                      // Turn left 90°
    usleep(PAUSE_TIME);
    
    move_until_obstacle(1);          // Forward until obstacle
    turn_90(0);                      // Turn right 90°
    usleep(PAUSE_TIME);
    
    move_until_obstacle(1);          // Forward until obstacle
    turn_90(0);                      // Turn right 90°
    usleep(PAUSE_TIME);
    
    move_until_obstacle(1);          // Forward until obstacle
    turn_90(1);                      // Turn left 90°
    usleep(PAUSE_TIME);
    
    move_forward_timed(FINAL_FORWARD_TIME);  // Forward for 3 seconds
    usleep(PAUSE_TIME);
    
    turn_180();                      // 180° turn
    usleep(PAUSE_TIME);
    
    // Reverse sequence without pauses
    printf("\n=== Reverse Sequence (no pauses) ===\n");
    move_forward_timed(FINAL_FORWARD_TIME);  // Forward for 3 seconds
    turn_90(0);                      // Turn right 90°
    move_until_obstacle(0);          // Forward until obstacle
    turn_90(1);                      // Turn left 90°
    move_until_obstacle(0);          // Forward until obstacle
    turn_90(1);                      // Turn left 90°
    move_until_obstacle(0);          // Forward until obstacle
    turn_90(0);                      // Turn right 90°
    move_until_obstacle(0);          // Forward until obstacle
    
    printf("\nDemo sequence completed!\n");
    
    pthread_join(sensor_thread, NULL);
    return EXIT_SUCCESS;
} 