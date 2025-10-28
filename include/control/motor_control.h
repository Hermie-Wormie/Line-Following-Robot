/**
 * @file motor_control.h
 * @brief Motor Control Driver for Robo PICO with Encoder Feedback - FIXED VERSION
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

// ============================================================================
// BASIC MOTOR CONTROL FUNCTIONS (NO PID)
// ============================================================================

/**
 * @brief Initialize motor control system
 * @return true if initialization successful
 */
bool motor_init(void);

/**
 * @brief Shutdown motor control system
 */
void motor_shutdown(void);

/**
 * @brief Stop both motors immediately
 */
void motor_stop(void);

/**
 * @brief Emergency stop (same as motor_stop but for clarity)
 */
void motor_emergency_stop(void);

/**
 * @brief Set motor speeds directly (percentage-based)
 * @param speed_left Left motor speed (-100 to +100, negative = backward)
 * @param speed_right Right motor speed (-100 to +100, negative = backward)
 * 
 * This is the simplest way to control motors without PID.
 * Example: motor_set_speed(50.0f, 50.0f) moves forward at 50% power
 */
void motor_set_speed(float speed_left, float speed_right);

/**
 * @brief Set motor PWM directly (alias for motor_set_speed)
 * @param pwm_left Left motor PWM duty cycle (-100 to +100)
 * @param pwm_right Right motor PWM duty cycle (-100 to +100)
 */
void motor_set_pwm(float pwm_left, float pwm_right);

/**
 * @brief Move forward at specified speed
 * @param speed Forward speed (0-100%)
 */
void motor_forward(float speed);

/**
 * @brief Move backward at specified speed
 * @param speed Backward speed (0-100%)
 */
void motor_backward(float speed);

/**
 * @brief Turn left (slower left wheel)
 * @param speed Base speed (0-100%)
 */
void motor_turn_left(float speed);

/**
 * @brief Turn right (slower right wheel)
 * @param speed Base speed (0-100%)
 */
void motor_turn_right(float speed);

/**
 * @brief Pivot turn in place
 * @param speed Turn speed (0-100%)
 * @param clockwise true for clockwise, false for counter-clockwise
 */
void motor_pivot_turn(float speed, bool clockwise);

/**
 * @brief Set individual motor
 * @param is_left true for left motor, false for right motor
 * @param direction Motor direction
 * @param speed Motor speed (0-100%)
 */
void motor_set(bool is_left, motor_direction_t direction, float speed);

// ============================================================================
// PID CONTROL FUNCTIONS (ADVANCED)
// ============================================================================

/**
 * @brief Enable or disable PID control
 * @param enable true to enable PID, false to disable
 * 
 * When PID is disabled (default), use motor_set_speed() for direct control.
 * When PID is enabled, use motor_set_speed_rpm() and call motor_update() periodically.
 */
void motor_pid_enable(bool enable);

/**
 * @brief Set PID controller parameters
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void motor_set_pid_params(float kp, float ki, float kd);

/**
 * @brief Set target speed in RPM (requires PID enabled)
 * @param rpm_left Target RPM for left motor
 * @param rpm_right Target RPM for right motor
 * 
 * Must enable PID first with motor_pid_enable(true)
 * Must call motor_update() periodically (every 100ms) for PID to work
 */
void motor_set_speed_rpm(float rpm_left, float rpm_right);

/**
 * @brief Update PID controllers (call every 100ms when PID enabled)
 * 
 * This function reads encoder speeds and adjusts motor PWM to reach target RPM.
 * Only call this when PID is enabled.
 */
void motor_update(void);

/**
 * @brief Reset PID controllers (clears integral and derivative terms)
 */
void motor_reset_pid(void);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Adjust motor balance (for drift correction)
 * @param correction Correction value (-50 to +50)
 */
void motor_adjust_balance(float correction);

/**
 * @brief Get motor state structure
 * @param is_left true for left motor, false for right motor
 * @return Pointer to motor state
 */
motor_state_t* motor_get_state(bool is_left);

/**
 * @brief Ramp speed smoothly over time
 * @param target_left Target left motor PWM
 * @param target_right Target right motor PWM
 * @param ramp_time_ms Ramp duration in milliseconds
 */
void motor_ramp_speed(float target_left, float target_right, uint32_t ramp_time_ms);

/**
 * @brief Get current PWM duty cycle
 * @param is_left true for left motor, false for right motor
 * @return Current PWM duty cycle (0-100%)
 */
float motor_get_pwm(bool is_left);

/**
 * @brief Run a test sequence to verify motor functionality
 */
void motor_test_sequence(void);

#endif // MOTOR_CONTROL_H