#include "control/pid_controller.h"
#include "pico/stdlib.h"
#include <stdio.h>

void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    
    pid->output_min = -100.0f;  // Default limits
    pid->output_max = 100.0f;
    
    pid->last_time = time_us_32();
    pid->initialized = true;
}

void pid_set_limits(pid_controller_t *pid, float min, float max) {
    if (!pid) return;
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint) {
    if (!pid) return;
    pid->setpoint = setpoint;
}

float pid_compute(pid_controller_t *pid, float measured_value) {
    if (!pid || !pid->initialized) return 0.0f;
    
    // Calculate time delta
    uint32_t current_time = time_us_32();
    float dt = (current_time - pid->last_time) / 1000000.0f;  // Convert to seconds
    pid->last_time = current_time;
    
    // Calculate error
    float error = pid->setpoint - measured_value;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term (with anti-windup)
    pid->integral += error * dt;
    
    // Anti-windup: clamp integral
    float max_integral = 50.0f;  // Prevent integral windup
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    
    // Save error for next iteration
    pid->prev_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

void pid_reset(pid_controller_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_time = time_us_32();
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_get_error(pid_controller_t *pid) {
    if (!pid) return 0.0f;
    return pid->prev_error;
}