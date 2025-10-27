#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include "pico/stdlib.h"
#include <stdbool.h>

/**
 * @file wheel_encoder.h
 * @brief Wheel encoder driver for speed and distance measurement
 * 
 * Uses optical encoders with A/B quadrature output to track wheel rotation.
 * Provides speed (RPM) and distance (mm) calculations.
 */

// Encoder identifiers
typedef enum {
    ENCODER_LEFT = 0,   // Motor 1 encoder
    ENCODER_RIGHT = 1   // Motor 2 encoder
} encoder_id_t;

// Encoder data structure
typedef struct {
    int32_t count;          // Total pulse count (signed for direction)
    float distance_mm;      // Total distance traveled in mm
    float speed_rpm;        // Current speed in RPM
    uint32_t last_update;   // Last update timestamp (microseconds)
    uint32_t pulse_period;  // Time between pulses (microseconds)
} encoder_data_t;

/**
 * @brief Initialize wheel encoder system
 * Sets up GPIO interrupts for both encoders
 */
void encoder_init(void);

/**
 * @brief Reset encoder count and distance for a specific encoder
 * @param encoder Encoder identifier
 */
void encoder_reset(encoder_id_t encoder);

/**
 * @brief Reset all encoders
 */
void encoder_reset_all(void);

/**
 * @brief Get current encoder count (pulse count)
 * @param encoder Encoder identifier
 * @return Pulse count (positive = forward, negative = backward)
 */
int32_t encoder_get_count(encoder_id_t encoder);

/**
 * @brief Get distance traveled in millimeters
 * @param encoder Encoder identifier
 * @return Distance in mm
 */
float encoder_get_distance_mm(encoder_id_t encoder);

/**
 * @brief Get current speed in RPM
 * @param encoder Encoder identifier
 * @return Speed in RPM
 */
float encoder_get_speed_rpm(encoder_id_t encoder);

/**
 * @brief Get current speed in mm/s
 * @param encoder Encoder identifier
 * @return Speed in mm/s
 */
float encoder_get_speed_mmps(encoder_id_t encoder);

/**
 * @brief Get complete encoder data structure
 * @param encoder Encoder identifier
 * @return Encoder data structure
 */
encoder_data_t encoder_get_data(encoder_id_t encoder);

/**
 * @brief Update speed calculations (call periodically)
 * Should be called at regular intervals for accurate speed measurement
 */
void encoder_update_speed(void);

/**
 * @brief Check if encoder system is initialized
 * @return true if initialized, false otherwise
 */
bool encoder_is_initialized(void);

/**
 * @brief Get average speed of both wheels in mm/s
 * @return Average speed
 */
float encoder_get_average_speed_mmps(void);

/**
 * @brief Get average distance of both wheels in mm
 * @return Average distance
 */
float encoder_get_average_distance_mm(void);

#endif // WHEEL_ENCODER_H