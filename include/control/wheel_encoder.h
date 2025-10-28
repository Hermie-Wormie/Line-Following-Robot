/**
 * @file wheel_encoder.h
 * @brief Single-channel wheel encoder driver for XYC-H206
 * 
 * Hardware: XYC-H206 IR wheel encoder (3-wire: VCC, GND, OUT)
 * Features:
 * - Speed measurement (RPM)
 * - Distance measurement (pulses and cm)
 * - Direction inference from motor commands
 * 
 * Note: Single-channel encoders cannot detect direction from hardware.
 * Direction is inferred from motor controller state.
 */

#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // For size_t

// Configuration constants
#define ENCODER_PULSES_PER_REVOLUTION 20  // XYC-H206 typical value
#define WHEEL_DIAMETER_CM 6.5             // Adjust to your wheel size
#define WHEEL_CIRCUMFERENCE_CM (3.14159 * WHEEL_DIAMETER_CM)

// Encoder direction (inferred from motor state)
typedef enum {
    ENCODER_DIR_FORWARD = 1,
    ENCODER_DIR_BACKWARD = -1,
    ENCODER_DIR_STOPPED = 0
} encoder_direction_t;

// Encoder data structure
typedef struct {
    // Raw pulse counting
    volatile int32_t pulse_count;      // Signed count (can be negative)
    volatile uint32_t total_pulses;    // Total pulses (always positive)
    
    // Direction inference
    encoder_direction_t direction;
    
    // Speed measurement
    volatile uint32_t last_pulse_time_us;
    volatile uint32_t pulse_interval_us;
    float rpm;
    float speed_cm_per_sec;
    
    // Distance tracking
    float distance_cm;
    
    // GPIO pin
    uint32_t gpio_pin;
    
    // Calibration
    bool is_calibrated;
    uint32_t pulses_per_rev;
} encoder_t;

/**
 * @brief Initialize a wheel encoder
 * 
 * @param encoder Pointer to encoder structure
 * @param gpio_pin GPIO pin connected to encoder OUT
 */
void encoder_init(encoder_t *encoder, uint32_t gpio_pin);

/**
 * @brief Set the direction for the encoder based on motor command
 * 
 * This must be called whenever you change motor direction!
 * 
 * @param encoder Pointer to encoder structure
 * @param direction Direction (FORWARD, BACKWARD, or STOPPED)
 */
void encoder_set_direction(encoder_t *encoder, encoder_direction_t direction);

/**
 * @brief Get current pulse count (signed, respects direction)
 * 
 * @param encoder Pointer to encoder structure
 * @return int32_t Pulse count (negative if moving backward)
 */
int32_t encoder_get_count(encoder_t *encoder);

/**
 * @brief Get total pulses (always positive, ignores direction)
 * 
 * @param encoder Pointer to encoder structure
 * @return uint32_t Total pulses counted
 */
uint32_t encoder_get_total_pulses(encoder_t *encoder);

/**
 * @brief Reset the pulse counter to zero
 * 
 * @param encoder Pointer to encoder structure
 */
void encoder_reset_count(encoder_t *encoder);

/**
 * @brief Calculate current speed in RPM
 * 
 * @param encoder Pointer to encoder structure
 * @return float Speed in RPM (0 if stopped or stale data)
 */
float encoder_get_rpm(encoder_t *encoder);

/**
 * @brief Calculate current speed in cm/s
 * 
 * @param encoder Pointer to encoder structure
 * @return float Speed in cm/s (negative if moving backward)
 */
float encoder_get_speed_cm_per_sec(encoder_t *encoder);

/**
 * @brief Get distance traveled in centimeters
 * 
 * @param encoder Pointer to encoder structure
 * @return float Distance in cm (negative if net backward movement)
 */
float encoder_get_distance_cm(encoder_t *encoder);

/**
 * @brief Reset distance measurement to zero
 * 
 * @param encoder Pointer to encoder structure
 */
void encoder_reset_distance(encoder_t *encoder);

/**
 * @brief Calibrate encoder pulses per revolution
 * 
 * Manually spin the wheel exactly one full rotation and call this
 * 
 * @param encoder Pointer to encoder structure
 * @param measured_pulses Number of pulses counted in one revolution
 */
void encoder_calibrate(encoder_t *encoder, uint32_t measured_pulses);

/**
 * @brief Update speed calculations (call periodically, e.g., 10Hz)
 * 
 * This checks if encoder has stopped (no recent pulses) and updates speed
 * 
 * @param encoder Pointer to encoder structure
 */
void encoder_update(encoder_t *encoder);

/**
 * @brief Get diagnostic information
 * 
 * @param encoder Pointer to encoder structure
 * @param buffer String buffer to write diagnostics
 * @param buffer_size Size of buffer
 */
void encoder_get_diagnostics(encoder_t *encoder, char *buffer, size_t buffer_size);

#endif // WHEEL_ENCODER_H