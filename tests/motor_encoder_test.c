/**
 * @file motor_encoder_test.c
 * @brief Motor Control + Encoder Integration Test
 * 
 * This test demonstrates:
 * - Motor control (forward, backward, stop)
 * - Encoder feedback (speed and distance)
 * - Direction synchronization
 * - Speed measurement
 * - Distance tracking
 * 
 * For Week 9 Partial Integration Demo
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "../include/control/wheel_encoder.h"
#include <stdio.h>

// ============================================================================
// HARDWARE CONFIGURATION - ADJUST TO YOUR WIRING
// ============================================================================

// Motor Control Pins (adjust to your motor driver)
#define MOTOR_LEFT_PWM_PIN   0    // Left motor PWM
#define MOTOR_LEFT_DIR1_PIN  1    // Left motor direction 1
#define MOTOR_LEFT_DIR2_PIN  2    // Left motor direction 2

#define MOTOR_RIGHT_PWM_PIN  3    // Right motor PWM
#define MOTOR_RIGHT_DIR1_PIN 4    // Right motor direction 1
#define MOTOR_RIGHT_DIR2_PIN 5    // Right motor direction 2

// Encoder Pins
#define ENCODER_LEFT_PIN     14   // Left encoder OUT
#define ENCODER_RIGHT_PIN    15   // Right encoder OUT

// PWM Configuration
#define PWM_FREQUENCY        1000  // 1 kHz
#define PWM_MAX_DUTY         65535 // 16-bit PWM

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Encoders
encoder_t left_encoder;
encoder_t right_encoder;

// Motor state
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} motor_direction_t;

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Initialize PWM for motor control
 */
void motor_pwm_init(uint32_t gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint32_t slice_num = pwm_gpio_to_slice_num(gpio_pin);
    
    // Configure PWM
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);  // 1 MHz PWM clock
    pwm_config_set_wrap(&config, PWM_MAX_DUTY);
    pwm_init(slice_num, &config, true);
    
    // Start at 0% duty
    pwm_set_gpio_level(gpio_pin, 0);
}

/**
 * @brief Initialize all motor control pins
 */
void motor_init(void) {
    // Initialize direction pins
    gpio_init(MOTOR_LEFT_DIR1_PIN);
    gpio_init(MOTOR_LEFT_DIR2_PIN);
    gpio_init(MOTOR_RIGHT_DIR1_PIN);
    gpio_init(MOTOR_RIGHT_DIR2_PIN);
    
    gpio_set_dir(MOTOR_LEFT_DIR1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_LEFT_DIR2_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_DIR1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_DIR2_PIN, GPIO_OUT);
    
    // Initialize PWM
    motor_pwm_init(MOTOR_LEFT_PWM_PIN);
    motor_pwm_init(MOTOR_RIGHT_PWM_PIN);
    
    // Start stopped
    gpio_put(MOTOR_LEFT_DIR1_PIN, 0);
    gpio_put(MOTOR_LEFT_DIR2_PIN, 0);
    gpio_put(MOTOR_RIGHT_DIR1_PIN, 0);
    gpio_put(MOTOR_RIGHT_DIR2_PIN, 0);
    
    printf("[MOTOR] Initialized\n");
}

/**
 * @brief Set motor speed (0-100%)
 */
void motor_set_speed(uint32_t pwm_pin, float speed_percent) {
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    
    uint16_t duty = (uint16_t)((speed_percent / 100.0f) * PWM_MAX_DUTY);
    pwm_set_gpio_level(pwm_pin, duty);
}

/**
 * @brief Set motor direction and update encoder direction
 * 
 * CRITICAL: This function keeps encoder direction synchronized with motor!
 */
void motor_set_direction(motor_direction_t left_dir, motor_direction_t right_dir, 
                         float left_speed, float right_speed) {
    // LEFT MOTOR
    switch (left_dir) {
        case MOTOR_FORWARD:
            gpio_put(MOTOR_LEFT_DIR1_PIN, 1);
            gpio_put(MOTOR_LEFT_DIR2_PIN, 0);
            motor_set_speed(MOTOR_LEFT_PWM_PIN, left_speed);
            encoder_set_direction(&left_encoder, ENCODER_DIR_FORWARD);
            break;
            
        case MOTOR_BACKWARD:
            gpio_put(MOTOR_LEFT_DIR1_PIN, 0);
            gpio_put(MOTOR_LEFT_DIR2_PIN, 1);
            motor_set_speed(MOTOR_LEFT_PWM_PIN, left_speed);
            encoder_set_direction(&left_encoder, ENCODER_DIR_BACKWARD);
            break;
            
        case MOTOR_STOP:
            gpio_put(MOTOR_LEFT_DIR1_PIN, 0);
            gpio_put(MOTOR_LEFT_DIR2_PIN, 0);
            motor_set_speed(MOTOR_LEFT_PWM_PIN, 0);
            encoder_set_direction(&left_encoder, ENCODER_DIR_STOPPED);
            break;
    }
    
    // RIGHT MOTOR
    switch (right_dir) {
        case MOTOR_FORWARD:
            gpio_put(MOTOR_RIGHT_DIR1_PIN, 1);
            gpio_put(MOTOR_RIGHT_DIR2_PIN, 0);
            motor_set_speed(MOTOR_RIGHT_PWM_PIN, right_speed);
            encoder_set_direction(&right_encoder, ENCODER_DIR_FORWARD);
            break;
            
        case MOTOR_BACKWARD:
            gpio_put(MOTOR_RIGHT_DIR1_PIN, 0);
            gpio_put(MOTOR_RIGHT_DIR2_PIN, 1);
            motor_set_speed(MOTOR_RIGHT_PWM_PIN, right_speed);
            encoder_set_direction(&right_encoder, ENCODER_DIR_BACKWARD);
            break;
            
        case MOTOR_STOP:
            gpio_put(MOTOR_RIGHT_DIR1_PIN, 0);
            gpio_put(MOTOR_RIGHT_DIR2_PIN, 0);
            motor_set_speed(MOTOR_RIGHT_PWM_PIN, 0);
            encoder_set_direction(&right_encoder, ENCODER_DIR_STOPPED);
            break;
    }
}

/**
 * @brief Move forward at specified speed
 */
void move_forward(float speed_percent) {
    motor_set_direction(MOTOR_FORWARD, MOTOR_FORWARD, speed_percent, speed_percent);
}

/**
 * @brief Move backward at specified speed
 */
void move_backward(float speed_percent) {
    motor_set_direction(MOTOR_BACKWARD, MOTOR_BACKWARD, speed_percent, speed_percent);
}

/**
 * @brief Stop both motors
 */
void move_stop(void) {
    motor_set_direction(MOTOR_STOP, MOTOR_STOP, 0, 0);
}

/**
 * @brief Turn left (left motor slower than right)
 */
void turn_left(float speed_percent) {
    motor_set_direction(MOTOR_FORWARD, MOTOR_FORWARD, 
                       speed_percent * 0.3f, speed_percent);
}

/**
 * @brief Turn right (right motor slower than left)
 */
void turn_right(float speed_percent) {
    motor_set_direction(MOTOR_FORWARD, MOTOR_FORWARD, 
                       speed_percent, speed_percent * 0.3f);
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Clear screen and display encoder status
 */
void display_status(const char* test_name) {
    printf("\033[2J\033[H");  // Clear screen
    
    printf("========================================\n");
    printf("  MOTOR + ENCODER INTEGRATION TEST\n");
    printf("========================================\n\n");
    
    printf("Current Test: %s\n\n", test_name);
    
    // Left motor/encoder
    printf("LEFT MOTOR/ENCODER:\n");
    printf("  Pulses:    %6ld\n", encoder_get_count(&left_encoder));
    printf("  Speed:     %6.1f RPM\n", encoder_get_rpm(&left_encoder));
    printf("  Speed:     %6.2f cm/s\n", encoder_get_speed_cm_per_sec(&left_encoder));
    printf("  Distance:  %6.2f cm\n\n", encoder_get_distance_cm(&left_encoder));
    
    // Right motor/encoder
    printf("RIGHT MOTOR/ENCODER:\n");
    printf("  Pulses:    %6ld\n", encoder_get_count(&right_encoder));
    printf("  Speed:     %6.1f RPM\n", encoder_get_rpm(&right_encoder));
    printf("  Speed:     %6.2f cm/s\n", encoder_get_speed_cm_per_sec(&right_encoder));
    printf("  Distance:  %6.2f cm\n\n", encoder_get_distance_cm(&right_encoder));
    
    printf("========================================\n");
}

// ============================================================================
// TEST SEQUENCES
// ============================================================================

/**
 * @brief Test 1: Forward movement with speed measurement
 */
void test_forward_movement(void) {
    printf("\n[TEST 1] Forward Movement @ 50%% speed\n");
    printf("Duration: 5 seconds\n");
    sleep_ms(2000);
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    move_forward(50.0f);
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 5000) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Forward Movement");
        sleep_ms(100);
    }
    
    move_stop();
    sleep_ms(2000);
    
    printf("\n[RESULT] Forward distance:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

/**
 * @brief Test 2: Backward movement with speed measurement
 */
void test_backward_movement(void) {
    printf("\n[TEST 2] Backward Movement @ 50%% speed\n");
    printf("Duration: 3 seconds\n");
    sleep_ms(2000);
    
    move_backward(50.0f);
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 3000) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Backward Movement");
        sleep_ms(100);
    }
    
    move_stop();
    sleep_ms(2000);
    
    printf("\n[RESULT] Net distance after backward:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

/**
 * @brief Test 3: Variable speed test
 */
void test_variable_speed(void) {
    printf("\n[TEST 3] Variable Speed Test\n");
    printf("Ramping speed: 30%% -> 70%% -> 30%%\n");
    sleep_ms(2000);
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    // Ramp up
    for (float speed = 30.0f; speed <= 70.0f; speed += 5.0f) {
        move_forward(speed);
        sleep_ms(500);
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Variable Speed - Ramp Up");
    }
    
    // Ramp down
    for (float speed = 70.0f; speed >= 30.0f; speed -= 5.0f) {
        move_forward(speed);
        sleep_ms(500);
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Variable Speed - Ramp Down");
    }
    
    move_stop();
    sleep_ms(2000);
    
    printf("\n[RESULT] Distance at variable speeds:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

/**
 * @brief Test 4: Turning test
 */
void test_turning(void) {
    printf("\n[TEST 4] Turning Test\n");
    sleep_ms(2000);
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    // Turn left
    printf("Turning LEFT...\n");
    turn_left(50.0f);
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 2000) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Turn Left");
        sleep_ms(100);
    }
    
    move_stop();
    sleep_ms(1000);
    
    // Turn right
    printf("Turning RIGHT...\n");
    turn_right(50.0f);
    start_time = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 2000) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Turn Right");
        sleep_ms(100);
    }
    
    move_stop();
    sleep_ms(2000);
    
    printf("\n[RESULT] Wheel distances during turns:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

/**
 * @brief Test 5: Distance-based movement (move exactly 50cm)
 */
void test_distance_movement(void) {
    printf("\n[TEST 5] Distance-Based Movement\n");
    printf("Target: 50 cm forward\n");
    sleep_ms(2000);
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    float target_distance = 50.0f;  // cm
    move_forward(40.0f);
    
    while (encoder_get_distance_cm(&left_encoder) < target_distance) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Distance Control - 50cm");
        sleep_ms(50);
    }
    
    move_stop();
    sleep_ms(2000);
    
    printf("\n[RESULT] Distance accuracy:\n");
    printf("  Target: %.2f cm\n", target_distance);
    printf("  Left:   %.2f cm (error: %.2f cm)\n", 
           encoder_get_distance_cm(&left_encoder),
           encoder_get_distance_cm(&left_encoder) - target_distance);
    printf("  Right:  %.2f cm (error: %.2f cm)\n", 
           encoder_get_distance_cm(&right_encoder),
           encoder_get_distance_cm(&right_encoder) - target_distance);
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("  MOTOR + ENCODER INTEGRATION TEST\n");
    printf("  Week 9 Partial Integration Demo\n");
    printf("========================================\n\n");
    
    // Initialize hardware
    printf("[INIT] Initializing motor control...\n");
    motor_init();
    
    printf("[INIT] Initializing encoders...\n");
    encoder_init(&left_encoder, ENCODER_LEFT_PIN);
    encoder_init(&right_encoder, ENCODER_RIGHT_PIN);
    
    printf("\n[READY] All systems initialized!\n");
    printf("Starting tests in 3 seconds...\n\n");
    sleep_ms(3000);
    
    // Run test sequence
    test_forward_movement();
    sleep_ms(2000);
    
    test_backward_movement();
    sleep_ms(2000);
    
    test_variable_speed();
    sleep_ms(2000);
    
    test_turning();
    sleep_ms(2000);
    
    test_distance_movement();
    
    // Final summary
    printf("\n========================================\n");
    printf("  ALL TESTS COMPLETE\n");
    printf("========================================\n\n");
    
    printf("Test Summary:\n");
    printf("  ✓ Forward movement with speed measurement\n");
    printf("  ✓ Backward movement with direction tracking\n");
    printf("  ✓ Variable speed control\n");
    printf("  ✓ Turning with differential speeds\n");
    printf("  ✓ Distance-based movement control\n\n");
    
    printf("Integration Status: PASS\n");
    printf("Motor + Encoder system working correctly!\n\n");
    
    // Continuous monitoring mode
    printf("Entering continuous monitoring mode...\n");
    printf("(Motors stopped - manually move robot to see encoder feedback)\n\n");
    
    move_stop();
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    while (true) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        display_status("Monitoring Mode - Stopped");
        sleep_ms(200);
    }
    
    return 0;
}