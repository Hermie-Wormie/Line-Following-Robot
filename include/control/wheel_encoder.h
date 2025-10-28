/**
 * @file wheel_encoder.h
 * @brief Wheel Encoder Driver for XCH206 Optical Encoders
 * 
 * This driver supports single-channel optical encoders (XCH206)
 * Each encoder uses one GPIO pin for pulse counting
 * 
 * Hardware Setup:
 * - Motor 1 Encoder: Connected to one Grove port
 * - Motor 2 Encoder: Connected to another Grove port (or sharing same port)
 * - Each XCH206 has: GND, VCC (3.3V or 5V), and one signal pin
 */

#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

// Default GPIO pins - modify these based on your wiring
#define ENCODER_LEFT_PIN    2   // GPIO pin for left wheel encoder
#define ENCODER_RIGHT_PIN   6   // GPIO pin for right wheel encoder

// Encoder configuration
#define ENCODER_PULSES_PER_REV  20  // XCH206 typically has 20 slots
#define WHEEL_DIAMETER_MM       65  // Adjust to your wheel diameter
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159)
#define MM_PER_PULSE           (WHEEL_CIRCUMFERENCE_MM / ENCODER_PULSES_PER_REV)

/**
 * @brief Encoder data structure
 */
typedef struct {
    uint32_t pulse_count;        // Total pulse count
    uint32_t last_pulse_time_us; // Time of last pulse (microseconds)
    float speed_rpm;             // Current speed in RPM
    float distance_mm;           // Distance traveled in mm
    bool direction_forward;      // True if moving forward
} encoder_data_t;

/**
 * @brief Initialize wheel encoders
 * 
 * @param left_pin GPIO pin number for left encoder
 * @param right_pin GPIO pin number for right encoder
 * @return true if initialization successful, false otherwise
 */
bool encoder_init(uint8_t left_pin, uint8_t right_pin);

/**
 * @brief Get encoder data for left wheel
 * 
 * @return Pointer to left encoder data structure
 */
encoder_data_t* encoder_get_left_data(void);

/**
 * @brief Get encoder data for right wheel
 * 
 * @return Pointer to right encoder data structure
 */
encoder_data_t* encoder_get_right_data(void);

/**
 * @brief Reset encoder counts and distance
 * 
 * @param reset_left Reset left encoder if true
 * @param reset_right Reset right encoder if true
 */
void encoder_reset(bool reset_left, bool reset_right);

/**
 * @brief Get speed in RPM for specified encoder
 * 
 * @param is_left True for left encoder, false for right
 * @return Speed in RPM
 */
float encoder_get_speed_rpm(bool is_left);

/**
 * @brief Get distance traveled in millimeters
 * 
 * @param is_left True for left encoder, false for right
 * @return Distance in mm
 */
float encoder_get_distance_mm(bool is_left);

/**
 * @brief Get pulse count
 * 
 * @param is_left True for left encoder, false for right
 * @return Total pulse count
 */
uint32_t encoder_get_pulse_count(bool is_left);

/**
 * @brief Update speed calculations (call periodically, e.g., every 100ms)
 * 
 * This function should be called regularly to update speed calculations
 * even when no pulses are detected (for detecting stopped wheels)
 */
void encoder_update_speed(void);

/**
 * @brief Set direction for encoder
 * 
 * @param is_left True for left encoder, false for right
 * @param forward True for forward direction, false for reverse
 */
void encoder_set_direction(bool is_left, bool forward);

#endif // WHEEL_ENCODER_H