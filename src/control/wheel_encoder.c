/**
 * @file wheel_encoder.c
 * @brief Wheel Encoder Driver Implementation for XCH206 - FIXED VERSION
 * 
 * Single-channel optical encoder driver for Raspberry Pi Pico
 * FIXED: Proper interrupt handling with single shared callback
 */

#include "wheel_encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>

// Encoder state
static encoder_data_t left_encoder = {0};
static encoder_data_t right_encoder = {0};
static uint8_t left_encoder_pin = ENCODER_LEFT_PIN;
static uint8_t right_encoder_pin = ENCODER_RIGHT_PIN;

// Speed calculation parameters
#define SPEED_TIMEOUT_US 500000  // 500ms - if no pulse, speed = 0
#define MIN_PULSE_INTERVAL_US 100 // Debounce: minimum 100us between pulses

/**
 * @brief Unified GPIO interrupt handler for both encoders
 * 
 * This is the CORRECT way to handle multiple GPIO interrupts on Pico
 */
static void encoder_gpio_callback(uint gpio, uint32_t events) {
    if (!(events & GPIO_IRQ_EDGE_RISE)) {
        return;  // Only handle rising edges
    }
    
    uint64_t current_time = time_us_64();
    encoder_data_t* encoder = NULL;
    
    // Determine which encoder triggered the interrupt
    if (gpio == left_encoder_pin) {
        encoder = &left_encoder;
    } else if (gpio == right_encoder_pin) {
        encoder = &right_encoder;
    } else {
        return;  // Unknown GPIO
    }
    
    uint64_t time_diff = current_time - encoder->last_pulse_time_us;
    
    // Debounce: ignore pulses that are too close together
    if (time_diff < MIN_PULSE_INTERVAL_US) {
        return;
    }
    
    // Update pulse count
    encoder->pulse_count++;
    
    // Calculate speed (RPM)
    // RPM = (60 * 1000000) / (time_per_pulse_us * pulses_per_rev)
    if (time_diff > 0) {
        encoder->speed_rpm = (60.0f * 1000000.0f) / 
                            ((float)time_diff * ENCODER_PULSES_PER_REV);
    }
    
    // Update distance
    encoder->distance_mm += MM_PER_PULSE;
    
    // Store timestamp
    encoder->last_pulse_time_us = current_time;
}

/**
 * @brief Initialize wheel encoders
 */
bool encoder_init(uint8_t left_pin, uint8_t right_pin) {
    // Store pin numbers
    left_encoder_pin = left_pin;
    right_encoder_pin = right_pin;
    
    printf("Initializing encoders - Left: GP%d, Right: GP%d\n", left_pin, right_pin);
    
    // Initialize encoder structures
    left_encoder.pulse_count = 0;
    left_encoder.last_pulse_time_us = 0;
    left_encoder.speed_rpm = 0.0f;
    left_encoder.distance_mm = 0.0f;
    left_encoder.direction_forward = true;
    
    right_encoder.pulse_count = 0;
    right_encoder.last_pulse_time_us = 0;
    right_encoder.speed_rpm = 0.0f;
    right_encoder.distance_mm = 0.0f;
    right_encoder.direction_forward = true;
    
    // Initialize GPIO for left encoder
    gpio_init(left_encoder_pin);
    gpio_set_dir(left_encoder_pin, GPIO_IN);
    gpio_pull_up(left_encoder_pin);  // Enable pull-up resistor
    
    // Initialize GPIO for right encoder
    gpio_init(right_encoder_pin);
    gpio_set_dir(right_encoder_pin, GPIO_IN);
    gpio_pull_up(right_encoder_pin);  // Enable pull-up resistor
    
    // Small delay to let pull-ups stabilize
    sleep_ms(10);
    
    // Set up interrupts - CRITICAL FIX: Use single callback for both
    // This is the proper way to handle multiple GPIO interrupts on Pico
    gpio_set_irq_enabled_with_callback(
        left_encoder_pin, 
        GPIO_IRQ_EDGE_RISE, 
        true, 
        &encoder_gpio_callback  // This callback handles BOTH pins
    );
    
    // Enable interrupt for right encoder (uses same callback)
    gpio_set_irq_enabled(
        right_encoder_pin, 
        GPIO_IRQ_EDGE_RISE, 
        true
    );
    
    printf("Encoder initialization complete!\n");
    printf("  Configuration: %d pulses per revolution\n", ENCODER_PULSES_PER_REV);
    printf("  Wheel diameter: %d mm\n", WHEEL_DIAMETER_MM);
    printf("  Distance per pulse: %.2f mm\n", MM_PER_PULSE);
    
    return true;
}

/**
 * @brief Get encoder data for left wheel
 */
encoder_data_t* encoder_get_left_data(void) {
    return &left_encoder;
}

/**
 * @brief Get encoder data for right wheel
 */
encoder_data_t* encoder_get_right_data(void) {
    return &right_encoder;
}

/**
 * @brief Reset encoder counts
 */
void encoder_reset(bool reset_left, bool reset_right) {
    if (reset_left) {
        left_encoder.pulse_count = 0;
        left_encoder.distance_mm = 0.0f;
        left_encoder.speed_rpm = 0.0f;
        left_encoder.last_pulse_time_us = 0;
        printf("Left encoder reset\n");
    }
    
    if (reset_right) {
        right_encoder.pulse_count = 0;
        right_encoder.distance_mm = 0.0f;
        right_encoder.speed_rpm = 0.0f;
        right_encoder.last_pulse_time_us = 0;
        printf("Right encoder reset\n");
    }
}

/**
 * @brief Get speed in RPM
 */
float encoder_get_speed_rpm(bool is_left) {
    return is_left ? left_encoder.speed_rpm : right_encoder.speed_rpm;
}

/**
 * @brief Get distance in millimeters
 */
float encoder_get_distance_mm(bool is_left) {
    encoder_data_t* encoder = is_left ? &left_encoder : &right_encoder;
    
    // Apply direction
    return encoder->direction_forward ? 
           encoder->distance_mm : -encoder->distance_mm;
}

/**
 * @brief Get pulse count
 */
uint32_t encoder_get_pulse_count(bool is_left) {
    return is_left ? left_encoder.pulse_count : right_encoder.pulse_count;
}

/**
 * @brief Update speed calculations
 * 
 * Call this periodically (e.g., every 100ms) to detect stopped wheels
 */
void encoder_update_speed(void) {
    uint64_t current_time = time_us_64();
    
    // Check left encoder
    if (current_time - left_encoder.last_pulse_time_us > SPEED_TIMEOUT_US) {
        left_encoder.speed_rpm = 0.0f;
    }
    
    // Check right encoder
    if (current_time - right_encoder.last_pulse_time_us > SPEED_TIMEOUT_US) {
        right_encoder.speed_rpm = 0.0f;
    }
}

/**
 * @brief Set direction for encoder
 */
void encoder_set_direction(bool is_left, bool forward) {
    if (is_left) {
        left_encoder.direction_forward = forward;
    } else {
        right_encoder.direction_forward = forward;
    }
}

/**
 * @brief Get diagnostic info for debugging
 */
void encoder_print_diagnostics(void) {
    printf("\n=== Encoder Diagnostics ===\n");
    printf("Left Encoder (GP%d):\n", left_encoder_pin);
    printf("  Pulses: %lu\n", left_encoder.pulse_count);
    printf("  Speed: %.2f RPM\n", left_encoder.speed_rpm);
    printf("  Distance: %.2f mm\n", left_encoder.distance_mm);
    printf("  Direction: %s\n", left_encoder.direction_forward ? "Forward" : "Backward");
    
    printf("Right Encoder (GP%d):\n", right_encoder_pin);
    printf("  Pulses: %lu\n", right_encoder.pulse_count);
    printf("  Speed: %.2f RPM\n", right_encoder.speed_rpm);
    printf("  Distance: %.2f mm\n", right_encoder.distance_mm);
    printf("  Direction: %s\n", right_encoder.direction_forward ? "Forward" : "Backward");
    printf("===========================\n\n");
}