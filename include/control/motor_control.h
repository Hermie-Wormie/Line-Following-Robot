#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"
#include <stdbool.h>

/**
 * @file motor_control.h
 * @brief Motor control driver for robotic car
 * 
 * Controls two DC motors using built-in motor driver (PWM + Direction pins)
 * Provides forward, backward, turning, and rotation functions.
 */

// Motor identifiers
typedef enum {
    MOTOR_LEFT = 0,   // Motor 1
    MOTOR_RIGHT = 1   // Motor 2
} motor_id_t;

// Motor direction
typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD = 1
} motor_direction_t;

/**
 * @brief Initialize motor control system
 * Sets up PWM and direction GPIO pins for both motors
 */
void motor_control_init(void);

/**
 * @brief Set motor speed and direction
 * @param motor Motor identifier (MOTOR_LEFT or MOTOR_RIGHT)
 * @param speed Speed percentage (-100 to 100, negative = backward)
 */
void motor_set_speed(motor_id_t motor, int speed);

/**
 * @brief Stop a specific motor
 * @param motor Motor identifier
 */
void motor_stop(motor_id_t motor);

/**
 * @brief Stop all motors
 */
void motor_stop_all(void);

/**
 * @brief Move forward at specified speed
 * @param speed Speed percentage (0-100)
 */
void motor_forward(int speed);

/**
 * @brief Move backward at specified speed
 * @param speed Speed percentage (0-100)
 */
void motor_backward(int speed);

/**
 * @brief Turn left (left motor slower/stopped, right motor forward)
 * @param speed Speed percentage (0-100)
 */
void motor_turn_left(int speed);

/**
 * @brief Turn right (right motor slower/stopped, left motor forward)
 * @param speed Speed percentage (0-100)
 */
void motor_turn_right(int speed);

/**
 * @brief Rotate left in place (left backward, right forward)
 * @param speed Speed percentage (0-100)
 */
void motor_rotate_left(int speed);

/**
 * @brief Rotate right in place (left forward, right backward)
 * @param speed Speed percentage (0-100)
 */
void motor_rotate_right(int speed);

/**
 * @brief Set differential motor speeds for precise control
 * Used by PID controller for line following
 * @param left_speed Left motor speed (-100 to 100)
 * @param right_speed Right motor speed (-100 to 100)
 */
void motor_set_differential(int left_speed, int right_speed);

/**
 * @brief Get current motor speed
 * @param motor Motor identifier
 * @return Current speed (-100 to 100)
 */
int motor_get_speed(motor_id_t motor);

/**
 * @brief Check if motor system is initialized
 * @return true if initialized
 */
bool motor_is_initialized(void);

#endif // MOTOR_CONTROL_H