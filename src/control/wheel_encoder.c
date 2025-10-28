/**
 * @file wheel_encoder.c
 * @brief Single-channel wheel encoder driver implementation for XYC-H206
 */

#include "control/wheel_encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>  // For size_t

// Speed timeout (if no pulse in this time, assume stopped)
#define SPEED_TIMEOUT_US 500000  // 0.5 seconds

// Global encoder pointers for IRQ handlers
static encoder_t *left_encoder_ptr = NULL;
static encoder_t *right_encoder_ptr = NULL;

/**
 * @brief GPIO interrupt handler for left encoder
 */
static void left_encoder_irq_handler(uint gpio, uint32_t events) {
    if (left_encoder_ptr == NULL) return;
    
    // Get current time
    uint32_t now = time_us_32();
    
    // Calculate pulse interval
    if (left_encoder_ptr->last_pulse_time_us != 0) {
        left_encoder_ptr->pulse_interval_us = now - left_encoder_ptr->last_pulse_time_us;
    }
    left_encoder_ptr->last_pulse_time_us = now;
    
    // Update counters based on direction
    left_encoder_ptr->pulse_count += left_encoder_ptr->direction;
    left_encoder_ptr->total_pulses++;
}

/**
 * @brief GPIO interrupt handler for right encoder
 */
static void right_encoder_irq_handler(uint gpio, uint32_t events) {
    if (right_encoder_ptr == NULL) return;
    
    // Get current time
    uint32_t now = time_us_32();
    
    // Calculate pulse interval
    if (right_encoder_ptr->last_pulse_time_us != 0) {
        right_encoder_ptr->pulse_interval_us = now - right_encoder_ptr->last_pulse_time_us;
    }
    right_encoder_ptr->last_pulse_time_us = now;
    
    // Update counters based on direction
    right_encoder_ptr->pulse_count += right_encoder_ptr->direction;
    right_encoder_ptr->total_pulses++;
}

void encoder_init(encoder_t *encoder, uint32_t gpio_pin) {
    // Initialize structure
    memset(encoder, 0, sizeof(encoder_t));
    encoder->gpio_pin = gpio_pin;
    encoder->direction = ENCODER_DIR_STOPPED;
    encoder->pulses_per_rev = ENCODER_PULSES_PER_REVOLUTION;
    encoder->is_calibrated = false;
    
    // Configure GPIO
    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_IN);
    gpio_pull_up(gpio_pin);  // Internal pull-up for optical encoder
    
    // Store pointer for IRQ (assumes max 2 encoders: left and right)
    if (left_encoder_ptr == NULL) {
        left_encoder_ptr = encoder;
        gpio_set_irq_enabled_with_callback(gpio_pin, GPIO_IRQ_EDGE_FALL, 
                                          true, &left_encoder_irq_handler);
    } else if (right_encoder_ptr == NULL) {
        right_encoder_ptr = encoder;
        gpio_set_irq_enabled_with_callback(gpio_pin, GPIO_IRQ_EDGE_FALL, 
                                          true, &right_encoder_irq_handler);
    }
    
    printf("[ENCODER] Initialized on GPIO %d\n", gpio_pin);
}

void encoder_set_direction(encoder_t *encoder, encoder_direction_t direction) {
    encoder->direction = direction;
}

int32_t encoder_get_count(encoder_t *encoder) {
    return encoder->pulse_count;
}

uint32_t encoder_get_total_pulses(encoder_t *encoder) {
    return encoder->total_pulses;
}

void encoder_reset_count(encoder_t *encoder) {
    encoder->pulse_count = 0;
    encoder->total_pulses = 0;
}

float encoder_get_rpm(encoder_t *encoder) {
    // Check if stopped (no recent pulse)
    uint32_t now = time_us_32();
    uint32_t time_since_pulse = now - encoder->last_pulse_time_us;
    
    if (time_since_pulse > SPEED_TIMEOUT_US || encoder->pulse_interval_us == 0) {
        encoder->rpm = 0.0f;
        return 0.0f;
    }
    
    // RPM calculation:
    // One pulse takes pulse_interval_us microseconds
    // One revolution takes (pulses_per_rev * pulse_interval_us) microseconds
    // RPM = (60,000,000 us/min) / (pulses_per_rev * pulse_interval_us)
    
    float revolutions_per_second = 1000000.0f / 
                                   (encoder->pulses_per_rev * encoder->pulse_interval_us);
    encoder->rpm = revolutions_per_second * 60.0f;
    
    return encoder->rpm;
}

float encoder_get_speed_cm_per_sec(encoder_t *encoder) {
    float rpm = encoder_get_rpm(encoder);
    
    if (rpm == 0.0f) {
        encoder->speed_cm_per_sec = 0.0f;
        return 0.0f;
    }
    
    // Convert RPM to cm/s
    // speed = (RPM / 60) * wheel_circumference
    encoder->speed_cm_per_sec = (rpm / 60.0f) * WHEEL_CIRCUMFERENCE_CM;
    
    // Apply direction sign
    if (encoder->direction == ENCODER_DIR_BACKWARD) {
        encoder->speed_cm_per_sec = -encoder->speed_cm_per_sec;
    }
    
    return encoder->speed_cm_per_sec;
}

float encoder_get_distance_cm(encoder_t *encoder) {
    // Distance = (pulse_count / pulses_per_rev) * wheel_circumference
    encoder->distance_cm = ((float)encoder->pulse_count / encoder->pulses_per_rev) 
                          * WHEEL_CIRCUMFERENCE_CM;
    return encoder->distance_cm;
}

void encoder_reset_distance(encoder_t *encoder) {
    encoder->pulse_count = 0;
    encoder->distance_cm = 0.0f;
}

void encoder_calibrate(encoder_t *encoder, uint32_t measured_pulses) {
    encoder->pulses_per_rev = measured_pulses;
    encoder->is_calibrated = true;
    printf("[ENCODER] Calibrated: %u pulses/revolution\n", measured_pulses);
}

void encoder_update(encoder_t *encoder) {
    // Check for timeout (stopped condition)
    uint32_t now = time_us_32();
    uint32_t time_since_pulse = now - encoder->last_pulse_time_us;
    
    if (time_since_pulse > SPEED_TIMEOUT_US) {
        encoder->rpm = 0.0f;
        encoder->speed_cm_per_sec = 0.0f;
    }
}

void encoder_get_diagnostics(encoder_t *encoder, char *buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size,
             "GPIO: %u | Pulses: %ld | Total: %lu | RPM: %.1f | Speed: %.2f cm/s | Distance: %.2f cm | Dir: %s",
             encoder->gpio_pin,
             encoder->pulse_count,
             encoder->total_pulses,
             encoder->rpm,
             encoder->speed_cm_per_sec,
             encoder->distance_cm,
             encoder->direction == ENCODER_DIR_FORWARD ? "FWD" :
             encoder->direction == ENCODER_DIR_BACKWARD ? "BWD" : "STOP");
}