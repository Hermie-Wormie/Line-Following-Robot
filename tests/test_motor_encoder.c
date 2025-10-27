#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "control/motor_control.h"
#include "control/wheel_encoder.h"
#include "common/robot_config.h"

/**
 * @file test_motor_encoder.c
 * @brief Comprehensive test program for motors and wheel encoders
 * 
 * Tests:
 * 1. Basic motor control (forward/backward)
 * 2. Encoder counting and direction
 * 3. Speed measurement
 * 4. Both motors together (synchronization)
 * 5. Distance measurement
 * 6. Turning maneuvers
 */

void print_separator(void) {
    printf("\n========================================\n");
}

void print_header(const char *title) {
    print_separator();
    printf("  %s\n", title);
    print_separator();
}

/**
 * Test 1: Basic Motor Control
 */
void test_motor_basic(void) {
    print_header("TEST 1: Basic Motor Control");
    
    // Test left motor forward
    printf("\n1. Left motor forward at 50%% for 2 seconds...\n");
    encoder_reset_all();
    motor_set_speed(MOTOR_LEFT, 50);
    sleep_ms(2000);
    motor_stop(MOTOR_LEFT);
    printf("   Left encoder count: %d\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Distance: %.2f mm\n", encoder_get_distance_mm(ENCODER_LEFT));
    sleep_ms(500);
    
    // Test left motor backward
    printf("\n2. Left motor backward at 50%% for 2 seconds...\n");
    encoder_reset_all();
    motor_set_speed(MOTOR_LEFT, -50);
    sleep_ms(2000);
    motor_stop(MOTOR_LEFT);
    printf("   Left encoder count: %d\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Distance: %.2f mm\n", encoder_get_distance_mm(ENCODER_LEFT));
    sleep_ms(500);
    
    // Test right motor forward
    printf("\n3. Right motor forward at 50%% for 2 seconds...\n");
    encoder_reset_all();
    motor_set_speed(MOTOR_RIGHT, 50);
    sleep_ms(2000);
    motor_stop(MOTOR_RIGHT);
    printf("   Right encoder count: %d\n", (int)encoder_get_count(ENCODER_RIGHT));
    printf("   Distance: %.2f mm\n", encoder_get_distance_mm(ENCODER_RIGHT));
    sleep_ms(500);
    
    // Test right motor backward
    printf("\n4. Right motor backward at 50%% for 2 seconds...\n");
    encoder_reset_all();
    motor_set_speed(MOTOR_RIGHT, -50);
    sleep_ms(2000);
    motor_stop(MOTOR_RIGHT);
    printf("   Right encoder count: %d\n", (int)encoder_get_count(ENCODER_RIGHT));
    printf("   Distance: %.2f mm\n", encoder_get_distance_mm(ENCODER_RIGHT));
    sleep_ms(500);
    
    printf("\n✓ Test 1 complete\n");
}

/**
 * Test 2: Both Motors Together
 */
void test_motors_together(void) {
    print_header("TEST 2: Both Motors Together");
    
    printf("Running both motors forward at 60%% for 3 seconds...\n\n");
    
    encoder_reset_all();
    motor_forward(60);
    sleep_ms(3000);
    motor_stop_all();
    
    int32_t left_count = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);
    float left_dist = encoder_get_distance_mm(ENCODER_LEFT);
    float right_dist = encoder_get_distance_mm(ENCODER_RIGHT);
    
    printf("Results:\n");
    printf("  Left:  %d counts, %.2f mm\n", (int)left_count, left_dist);
    printf("  Right: %d counts, %.2f mm\n", (int)right_count, right_dist);
    printf("  Difference: %d counts, %.2f mm\n", 
           (int)abs(left_count - right_count),
           fabsf(left_dist - right_dist));
    
    float avg_dist = (left_dist + right_dist) / 2.0f;
    float sync_error = 0.0f;
    if (avg_dist > 0.1f) {  // Avoid division by zero
        sync_error = fabsf(left_dist - right_dist) / avg_dist * 100.0f;
    }
    printf("  Synchronization error: %.1f%%\n", sync_error);
    
    if (sync_error < 10.0f) {
        printf("  ✓ Good synchronization\n");
    } else {
        printf("  ⚠ Motors may need calibration\n");
    }
    
    printf("\n✓ Test 2 complete\n");
    sleep_ms(500);
}

/**
 * Test 3: Speed Variations
 */
void test_speed_variations(void) {
    print_header("TEST 3: Speed Variations");
    
    int speeds[] = {25, 50, 75, 100};
    int num_speeds = sizeof(speeds) / sizeof(speeds[0]);
    
    for (int i = 0; i < num_speeds; i++) {
        printf("\nTesting at %d%% speed for 1.5 seconds...\n", speeds[i]);
        encoder_reset_all();
        
        motor_forward(speeds[i]);
        sleep_ms(1500);
        motor_stop_all();
        
        printf("  Left:  %d counts, %.2f mm\n", 
               (int)encoder_get_count(ENCODER_LEFT),
               encoder_get_distance_mm(ENCODER_LEFT));
        printf("  Right: %d counts, %.2f mm\n",
               (int)encoder_get_count(ENCODER_RIGHT),
               encoder_get_distance_mm(ENCODER_RIGHT));
        
        sleep_ms(500);
    }
    
    printf("\n✓ Test 3 complete\n");
}

/**
 * Test 4: Speed Measurement
 */
void test_speed_measurement(void) {
    print_header("TEST 4: Speed Measurement");
    
    printf("Running at 70%% for 5 seconds with live speed updates\n\n");
    
    encoder_reset_all();
    motor_forward(70);
    
    printf("Time | Left RPM | Right RPM | Left mm/s | Right mm/s\n");
    printf("-----|----------|-----------|-----------|------------\n");
    
    for (int i = 0; i < 10; i++) {
        sleep_ms(500);
        encoder_update_speed();
        
        float left_rpm = encoder_get_speed_rpm(ENCODER_LEFT);
        float right_rpm = encoder_get_speed_rpm(ENCODER_RIGHT);
        float left_mmps = encoder_get_speed_mmps(ENCODER_LEFT);
        float right_mmps = encoder_get_speed_mmps(ENCODER_RIGHT);
        
        printf("%3ds | %8.1f | %9.1f | %9.1f | %10.1f\n",
               (i + 1) / 2, left_rpm, right_rpm, left_mmps, right_mmps);
    }
    
    motor_stop_all();
    
    printf("\nFinal distance traveled:\n");
    printf("  Left:  %.2f mm\n", encoder_get_distance_mm(ENCODER_LEFT));
    printf("  Right: %.2f mm\n", encoder_get_distance_mm(ENCODER_RIGHT));
    printf("  Average: %.2f mm\n", encoder_get_average_distance_mm());
    
    printf("\n✓ Test 4 complete\n");
}

/**
 * Test 5: Turning Tests
 */
void test_turning(void) {
    print_header("TEST 5: Turning Tests");
    
    // Test turn left
    printf("\n1. Turn left for 1 second...\n");
    encoder_reset_all();
    motor_turn_left(60);
    sleep_ms(1000);
    motor_stop_all();
    printf("   Left:  %d counts\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Right: %d counts\n", (int)encoder_get_count(ENCODER_RIGHT));
    sleep_ms(500);
    
    // Test turn right
    printf("\n2. Turn right for 1 second...\n");
    encoder_reset_all();
    motor_turn_right(60);
    sleep_ms(1000);
    motor_stop_all();
    printf("   Left:  %d counts\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Right: %d counts\n", (int)encoder_get_count(ENCODER_RIGHT));
    sleep_ms(500);
    
    // Test rotate left
    printf("\n3. Rotate left (in place) for 1 second...\n");
    encoder_reset_all();
    motor_rotate_left(50);
    sleep_ms(1000);
    motor_stop_all();
    printf("   Left:  %d counts (should be negative)\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Right: %d counts (should be positive)\n", (int)encoder_get_count(ENCODER_RIGHT));
    sleep_ms(500);
    
    // Test rotate right
    printf("\n4. Rotate right (in place) for 1 second...\n");
    encoder_reset_all();
    motor_rotate_right(50);
    sleep_ms(1000);
    motor_stop_all();
    printf("   Left:  %d counts (should be positive)\n", (int)encoder_get_count(ENCODER_LEFT));
    printf("   Right: %d counts (should be negative)\n", (int)encoder_get_count(ENCODER_RIGHT));
    
    printf("\n✓ Test 5 complete\n");
}

/**
 * Diagnostic Summary
 */
void print_diagnostic_summary(void) {
    print_header("DIAGNOSTIC SUMMARY");
    
    printf("\n✓ What to check:\n\n");
    printf("1. Encoder Counts:\n");
    printf("   - Should be POSITIVE when moving forward\n");
    printf("   - Should be NEGATIVE when moving backward\n");
    printf("   - If counts are zero, check encoder connections\n\n");
    
    printf("2. Motor Synchronization:\n");
    printf("   - Both motors should have similar counts at same speed\n");
    printf("   - If difference > 20%%, check:\n");
    printf("     * Motor mounting/alignment\n");
    printf("     * Wheel friction\n");
    printf("     * Encoder disc alignment\n\n");
    
    printf("3. Speed Consistency:\n");
    printf("   - Higher speeds should produce more counts\n");
    printf("   - Speed should be roughly proportional to PWM\n\n");
    
    printf("4. Direction:\n");
    printf("   - Forward = positive counts\n");
    printf("   - Backward = negative counts\n");
    printf("   - If reversed, swap encoder A/B pins\n\n");
    
    printf("5. Distance Calculation:\n");
    printf("   - Verify wheel circumference in robot_config.h\n");
    printf("   - Verify encoder pulses per revolution\n");
    printf("   - Current: %.2f mm per pulse\n\n", 
           WHEEL_CIRCUMFERENCE_MM / ENCODER_PULSES_PER_REV);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial connection
    
    print_header("MOTOR & ENCODER TEST PROGRAM");
    printf("\nHardware Configuration:\n");
    printf("  Left Motor:  PWM=GP%d, DIR=GP%d\n", MOTOR1_PWM_PIN, MOTOR1_DIR_PIN);
    printf("  Right Motor: PWM=GP%d, DIR=GP%d\n", MOTOR2_PWM_PIN, MOTOR2_DIR_PIN);
    printf("  Left Encoder:  A=GP%d, B=GP%d\n", MOTOR1_ENCODER_A_PIN, MOTOR1_ENCODER_B_PIN);
    printf("  Right Encoder: A=GP%d, B=GP%d\n", MOTOR2_ENCODER_A_PIN, MOTOR2_ENCODER_B_PIN);
    print_separator();
    
    // Initialize hardware
    printf("\nInitializing motors and encoders...\n");
    motor_control_init();
    encoder_init();
    printf("Initialization complete!\n");
    
    sleep_ms(2000);
    
    // Run all tests
    test_motor_basic();
    sleep_ms(1000);
    
    test_motors_together();
    sleep_ms(1000);
    
    test_speed_variations();
    sleep_ms(1000);
    
    test_speed_measurement();
    sleep_ms(1000);
    
    test_turning();
    sleep_ms(1000);
    
    print_diagnostic_summary();
    
    print_header("ALL TESTS COMPLETE!");
    printf("\nRobot is ready for integration testing.\n");
    printf("Next steps:\n");
    printf("  1. Test PID controller (test_motor_pid)\n");
    printf("  2. Integrate with line following (test_line_motor)\n");
    printf("  3. Full system integration (robot_main)\n");
    print_separator();
    
    // Ensure motors are stopped
    motor_stop_all();
    
    return 0;
}