#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

/**
 * @file pid_controller.h
 * @brief PID (Proportional-Integral-Derivative) controller implementation
 * 
 * Provides PID control for line following and motor speed control.
 */

// PID controller structure
typedef struct {
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
    
    float setpoint;    // Desired value
    float integral;    // Accumulated integral
    float prev_error;  // Previous error for derivative
    
    float output_min;  // Minimum output limit
    float output_max;  // Maximum output limit
    
    uint32_t last_time; // Last update time (microseconds)
    bool initialized;
} pid_controller_t;

/**
 * @brief Initialize PID controller
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Set output limits for PID controller
 * @param pid Pointer to PID controller
 * @param min Minimum output value
 * @param max Maximum output value
 */
void pid_set_limits(pid_controller_t *pid, float min, float max);

/**
 * @brief Set setpoint (target value) for PID controller
 * @param pid Pointer to PID controller
 * @param setpoint Desired value
 */
void pid_set_setpoint(pid_controller_t *pid, float setpoint);

/**
 * @brief Compute PID output
 * @param pid Pointer to PID controller
 * @param measured_value Current measured value
 * @return PID output value (limited by min/max)
 */
float pid_compute(pid_controller_t *pid, float measured_value);

/**
 * @brief Reset PID controller (clears integral and derivative terms)
 * @param pid Pointer to PID controller
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief Update PID gains
 * @param pid Pointer to PID controller
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Get current error
 * @param pid Pointer to PID controller
 * @return Current error (setpoint - measured_value)
 */
float pid_get_error(pid_controller_t *pid);

#endif // PID_CONTROLLER_H