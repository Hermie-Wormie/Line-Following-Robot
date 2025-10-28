/**
 * @file motor_control.h
 * @brief Motor Control Driver for Robo PICO with Encoder Feedback
 * 
 * Hardware: Robo PICO with onboard motor drivers
 * - Motor 1 (Left): M1A/M1B - GP8 (PWM), GP9 (DIR)
 * - Motor 2 (Right): M2A/M2B - GP10 (PWM), GP11 (DIR)
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Motor GPIO pins - Robo PICO configuration
#define MOTOR_LEFT_PWM_PIN      8   // GP8 - Motor 1 PWM
#define MOTOR_LEFT_DIR_PIN      9   // GP9 - Motor 1 Direction
#define MOTOR_RIGHT_PWM_PIN     10  // GP10 - Motor 2 PWM  
#define MOTOR_RIGHT_DIR_PIN     11  // GP11 - Motor 2 Direction

// PWM configuration
#define PWM_FREQUENCY_HZ        1000    // 1kHz PWM frequency
#define PWM_WRAP_VALUE          12499   // For 1kHz at 125MHz system clock
#define PWM_MAX_DUTY            100     // 100% duty cycle

// Speed limits
#define MIN_PWM_DUTY            15      // Minimum duty cycle (15%)
#define MAX_PWM_DUTY            100     // Maximum duty cycle (100%)
#define TURN_SPEED_FACTOR       0.7f    // Speed reduction for turns (70%)

// PID controller parameters
#define PID_KP_DEFAULT          0.8f    // Proportional gain
#define PID_KI_DEFAULT          0.1f    // Integral gain
#define PID_KD_DEFAULT          0.05f   // Derivative gain
#define PID_INTEGRAL_MAX        100.0f  // Anti-windup limit
#define PID_OUTPUT_MAX          50.0f   // Maximum PID output

/**
 * @brief Motor direction enum
 */
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} motor_direction_t;

/**
 * @brief PID controller structure
 */
typedef struct {
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float integral;         // Integral accumulator
    float last_error;       // Previous error for derivative
    float setpoint;         // Target value (RPM)
    float output;           // PID output
    bool enabled;           // PID enabled flag
} pid_controller_t;

/**
 * @brief Motor state structure
 */
typedef struct {
    float current_pwm;      // Current PWM duty cycle (0-100)
    float target_rpm;       // Target speed in RPM
    float current_rpm;      // Current speed from encoder
    motor_direction_t direction;
    pid_controller_t pid;
    bool enabled;
} motor_state_t;

// Basic motor control functions
bool motor_init(void);
void motor_shutdown(void);
void motor_stop(void);
void motor_emergency_stop(void);

// Direct PWM control (no PID)
void motor_set_pwm(float pwm_left, float pwm_right);
void motor_forward(float speed);
void motor_backward(float speed);
void motor_turn_left(float speed);
void motor_turn_right(float speed);
void motor_pivot_turn(float speed, bool clockwise);
void motor_set(bool is_left, motor_direction_t direction, float speed);

// PID control functions (for advanced use)
void motor_pid_enable(bool enable);
void motor_set_pid_params(float kp, float ki, float kd);
void motor_set_speed_rpm(float rpm_left, float rpm_right);
void motor_update(void);
void motor_reset_pid(void);

// Utility functions
void motor_adjust_balance(float correction);
motor_state_t* motor_get_state(bool is_left);
void motor_ramp_speed(float target_left, float target_right, uint32_t ramp_time_ms);
float motor_get_pwm(bool is_left);
void motor_test_sequence(void);

#endif // MOTOR_CONTROL_H