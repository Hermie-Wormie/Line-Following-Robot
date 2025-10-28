/**
 * @file wheel_encoder.c
 * @brief Wheel Encoder Driver Implementation for XCH206
 * 
 * Single-channel optical encoder driver for Raspberry Pi Pico
 */

#include "wheel_encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Encoder state
static encoder_data_t left_encoder = {0};
static encoder_data_t right_encoder = {0};
static uint8_t left_encoder_pin = ENCODER_LEFT_PIN;
static uint8_t right_encoder_pin = ENCODER_RIGHT_PIN;

// Speed calculation parameters
#define SPEED_TIMEOUT_US 500000  // 500ms - if no pulse, speed = 0
#define MIN_PULSE_INTERVAL_US 100 // Debounce: minimum 100us between pulses

/**
 * @brief GPIO interrupt handler for left encoder
 */
static void left_encoder_isr(uint gpio, uint32_t events) {
    (void)gpio; // Unused parameter
    
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t current_time = time_us_64();
        uint64_t time_diff = current_time - left_encoder.last_pulse_time_us;
        
        // Debounce: ignore pulses that are too close together
        if (time_diff < MIN_PULSE_INTERVAL_US) {
            return;
        }
        
        // Update pulse count
        left_encoder.pulse_count++;
        
        // Calculate speed (RPM)
        // RPM = (60 * 1000000) / (time_per_pulse_us * pulses_per_rev)
        if (time_diff > 0) {
            left_encoder.speed_rpm = (60.0f * 1000000.0f) / 
                                     ((float)time_diff * ENCODER_PULSES_PER_REV);
        }
        
        // Update distance
        left_encoder.distance_mm += MM_PER_PULSE;
        
        // Store timestamp
        left_encoder.last_pulse_time_us = current_time;
    }
}

/**
 * @brief GPIO interrupt handler for right encoder
 */
static void right_encoder_isr(uint gpio, uint32_t events) {
    (void)gpio; // Unused parameter
    
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint64_t current_time = time_us_64();
        uint64_t time_diff = current_time - right_encoder.last_pulse_time_us;
        
        // Debounce
        if (time_diff < MIN_PULSE_INTERVAL_US) {
            return;
        }
        
        // Update pulse count
        right_encoder.pulse_count++;
        
        // Calculate speed
        if (time_diff > 0) {
            right_encoder.speed_rpm = (60.0f * 1000000.0f) / 
                                      ((float)time_diff * ENCODER_PULSES_PER_REV);
        }
        
        // Update distance
        right_encoder.distance_mm += MM_PER_PULSE;
        
        // Store timestamp
        right_encoder.last_pulse_time_us = current_time;
    }
}

/**
 * @brief Initialize wheel encoders
 */
bool encoder_init(uint8_t left_pin, uint8_t right_pin) {
    // Store pin numbers
    left_encoder_pin = left_pin;
    right_encoder_pin = right_pin;
    
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
    
    // Set up interrupts for rising edge
    gpio_set_irq_enabled_with_callback(left_encoder_pin, 
                                       GPIO_IRQ_EDGE_RISE, 
                                       true, 
                                       &left_encoder_isr);
    
    gpio_set_irq_enabled(right_encoder_pin, 
                        GPIO_IRQ_EDGE_RISE, 
                        true);
    
    // Register separate callback for right encoder
    // Note: Pico SDK allows only one callback per GPIO, but we handle both in ISR
    gpio_add_raw_irq_handler(right_encoder_pin, &right_encoder_isr);
    
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
    }
    
    if (reset_right) {
        right_encoder.pulse_count = 0;
        right_encoder.distance_mm = 0.0f;
        right_encoder.speed_rpm = 0.0f;
        right_encoder.last_pulse_time_us = 0;
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