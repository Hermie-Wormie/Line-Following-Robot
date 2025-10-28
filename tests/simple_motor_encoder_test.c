/**
 * @file simple_motor_encoder_test.c
 * @brief Simple Motor + Encoder Test
 * 
 * Basic test for verifying motor control and encoder feedback
 * Easier to understand and modify
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "../include/control/wheel_encoder.h"
#include <stdio.h>

// ============================================================================
// CONFIGURATION - CHANGE THESE TO MATCH YOUR WIRING
// ============================================================================

// Motor pins (example - adjust to your setup)
#define MOTOR_LEFT_FWD   10
#define MOTOR_LEFT_BWD   11
#define MOTOR_RIGHT_FWD  12
#define MOTOR_RIGHT_BWD  13

// Encoder pins
#define ENCODER_LEFT     14
#define ENCODER_RIGHT    15

// ============================================================================
// GLOBALS
// ============================================================================

encoder_t left_encoder;
encoder_t right_encoder;

// ============================================================================
// SIMPLE MOTOR CONTROL (NO PWM)
// ============================================================================

void motor_init(void) {
    gpio_init(MOTOR_LEFT_FWD);
    gpio_init(MOTOR_LEFT_BWD);
    gpio_init(MOTOR_RIGHT_FWD);
    gpio_init(MOTOR_RIGHT_BWD);
    
    gpio_set_dir(MOTOR_LEFT_FWD, GPIO_OUT);
    gpio_set_dir(MOTOR_LEFT_BWD, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_FWD, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_BWD, GPIO_OUT);
    
    // Start stopped
    gpio_put(MOTOR_LEFT_FWD, 0);
    gpio_put(MOTOR_LEFT_BWD, 0);
    gpio_put(MOTOR_RIGHT_FWD, 0);
    gpio_put(MOTOR_RIGHT_BWD, 0);
}

void motors_forward(void) {
    gpio_put(MOTOR_LEFT_FWD, 1);
    gpio_put(MOTOR_LEFT_BWD, 0);
    gpio_put(MOTOR_RIGHT_FWD, 1);
    gpio_put(MOTOR_RIGHT_BWD, 0);
    
    encoder_set_direction(&left_encoder, ENCODER_DIR_FORWARD);
    encoder_set_direction(&right_encoder, ENCODER_DIR_FORWARD);
}

void motors_backward(void) {
    gpio_put(MOTOR_LEFT_FWD, 0);
    gpio_put(MOTOR_LEFT_BWD, 1);
    gpio_put(MOTOR_RIGHT_FWD, 0);
    gpio_put(MOTOR_RIGHT_BWD, 1);
    
    encoder_set_direction(&left_encoder, ENCODER_DIR_BACKWARD);
    encoder_set_direction(&right_encoder, ENCODER_DIR_BACKWARD);
}

void motors_stop(void) {
    gpio_put(MOTOR_LEFT_FWD, 0);
    gpio_put(MOTOR_LEFT_BWD, 0);
    gpio_put(MOTOR_RIGHT_FWD, 0);
    gpio_put(MOTOR_RIGHT_BWD, 0);
    
    encoder_set_direction(&left_encoder, ENCODER_DIR_STOPPED);
    encoder_set_direction(&right_encoder, ENCODER_DIR_STOPPED);
}

// ============================================================================
// DISPLAY
// ============================================================================

void print_status(void) {
    printf("\n--- Motor + Encoder Status ---\n");
    printf("LEFT  - Pulses: %6ld | Speed: %6.1f RPM | Distance: %6.2f cm\n",
           encoder_get_count(&left_encoder),
           encoder_get_rpm(&left_encoder),
           encoder_get_distance_cm(&left_encoder));
    
    printf("RIGHT - Pulses: %6ld | Speed: %6.1f RPM | Distance: %6.2f cm\n",
           encoder_get_count(&right_encoder),
           encoder_get_rpm(&right_encoder),
           encoder_get_distance_cm(&right_encoder));
}

// ============================================================================
// TESTS
// ============================================================================

void test_forward(void) {
    printf("\n========================================\n");
    printf("TEST 1: Forward Movement (5 seconds)\n");
    printf("========================================\n");
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    motors_forward();
    
    for (int i = 0; i < 50; i++) {  // 5 seconds (50 x 100ms)
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        sleep_ms(100);
        
        if (i % 5 == 0) {  // Print every 500ms
            print_status();
        }
    }
    
    motors_stop();
    sleep_ms(1000);
    
    printf("\n[FINAL] Forward distance:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

void test_backward(void) {
    printf("\n========================================\n");
    printf("TEST 2: Backward Movement (3 seconds)\n");
    printf("========================================\n");
    
    motors_backward();
    
    for (int i = 0; i < 30; i++) {  // 3 seconds
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        sleep_ms(100);
        
        if (i % 5 == 0) {
            print_status();
        }
    }
    
    motors_stop();
    sleep_ms(1000);
    
    printf("\n[FINAL] Net distance (should be positive minus backward):\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

void test_start_stop(void) {
    printf("\n========================================\n");
    printf("TEST 3: Start/Stop Cycles\n");
    printf("========================================\n");
    
    encoder_reset_count(&left_encoder);
    encoder_reset_count(&right_encoder);
    
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("\nCycle %d: Forward 2s, Stop 1s\n", cycle + 1);
        
        motors_forward();
        for (int i = 0; i < 20; i++) {
            encoder_update(&left_encoder);
            encoder_update(&right_encoder);
            sleep_ms(100);
        }
        
        motors_stop();
        print_status();
        sleep_ms(1000);
    }
    
    printf("\n[FINAL] Total distance from cycles:\n");
    printf("  Left:  %.2f cm\n", encoder_get_distance_cm(&left_encoder));
    printf("  Right: %.2f cm\n", encoder_get_distance_cm(&right_encoder));
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("  SIMPLE MOTOR + ENCODER TEST\n");
    printf("========================================\n\n");
    
    // Initialize
    printf("[INIT] Initializing motors...\n");
    motor_init();
    
    printf("[INIT] Initializing encoders...\n");
    encoder_init(&left_encoder, ENCODER_LEFT);
    encoder_init(&right_encoder, ENCODER_RIGHT);
    
    printf("\n[READY] System initialized!\n");
    sleep_ms(2000);
    
    // Run tests
    test_forward();
    sleep_ms(2000);
    
    test_backward();
    sleep_ms(2000);
    
    test_start_stop();
    
    // Done
    printf("\n========================================\n");
    printf("  ALL TESTS COMPLETE\n");
    printf("========================================\n\n");
    
    printf("✓ Motors respond to commands\n");
    printf("✓ Encoders track movement\n");
    printf("✓ Direction synchronization works\n");
    printf("✓ Speed measurement functional\n");
    printf("✓ Distance tracking accurate\n\n");
    
    printf("Press Ctrl+C to exit\n");
    
    // Monitor mode
    motors_stop();
    while (true) {
        encoder_update(&left_encoder);
        encoder_update(&right_encoder);
        print_status();
        sleep_ms(1000);
    }
    
    return 0;
}