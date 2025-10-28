/**
 * @file motor_control.c
 * @brief Motor Control Implementation for Robo PICO
 */

#include "motor_control.h"
#include "wheel_encoder.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>

// Motor state
static motor_state_t left_motor = {0};
static motor_state_t right_motor = {0};

// PWM slice numbers
static uint left_pwm_slice;
static uint right_pwm_slice;

// Forward declarations
static void set_motor_direction(bool is_left, motor_direction_t direction);
static void set_motor_pwm_duty(bool is_left, float duty);
static float compute_pid(pid_controller_t* pid, float current_value);
static float constrain_float(float value, float min, float max);

/**
 * @brief Initialize motor control system
 */
bool motor_init(void) {
    // Initialize motor state structures
    left_motor.current_pwm = 0.0f;
    left_motor.target_rpm = 0.0f;
    left_motor.current_rpm = 0.0f;
    left_motor.direction = MOTOR_STOP;
    left_motor.enabled = true;
    
    // Initialize left PID
    left_motor.pid.kp = PID_KP_DEFAULT;
    left_motor.pid.ki = PID_KI_DEFAULT;
    left_motor.pid.kd = PID_KD_DEFAULT;
    left_motor.pid.integral = 0.0f;
    left_motor.pid.last_error = 0.0f;
    left_motor.pid.setpoint = 0.0f;
    left_motor.pid.output = 0.0f;
    left_motor.pid.enabled = false;  // Disabled by default
    
    // Initialize right motor
    right_motor.current_pwm = 0.0f;
    right_motor.target_rpm = 0.0f;
    right_motor.current_rpm = 0.0f;
    right_motor.direction = MOTOR_STOP;
    right_motor.enabled = true;
    
    right_motor.pid.kp = PID_KP_DEFAULT;
    right_motor.pid.ki = PID_KI_DEFAULT;
    right_motor.pid.kd = PID_KD_DEFAULT;
    right_motor.pid.integral = 0.0f;
    right_motor.pid.last_error = 0.0f;
    right_motor.pid.setpoint = 0.0f;
    right_motor.pid.output = 0.0f;
    right_motor.pid.enabled = false;  // Disabled by default
    
    // Initialize direction pins (single pin per motor on Robo PICO)
    gpio_init(MOTOR_LEFT_DIR_PIN);
    gpio_set_dir(MOTOR_LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR_LEFT_DIR_PIN, 0);
    
    gpio_init(MOTOR_RIGHT_DIR_PIN);
    gpio_set_dir(MOTOR_RIGHT_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_DIR_PIN, 0);
    
    // Initialize PWM for left motor
    gpio_set_function(MOTOR_LEFT_PWM_PIN, GPIO_FUNC_PWM);
    left_pwm_slice = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM_PIN);
    pwm_set_wrap(left_pwm_slice, PWM_WRAP_VALUE);
    pwm_set_chan_level(left_pwm_slice, pwm_gpio_to_channel(MOTOR_LEFT_PWM_PIN), 0);
    pwm_set_enabled(left_pwm_slice, true);
    
    // Initialize PWM for right motor
    gpio_set_function(MOTOR_RIGHT_PWM_PIN, GPIO_FUNC_PWM);
    right_pwm_slice = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM_PIN);
    pwm_set_wrap(right_pwm_slice, PWM_WRAP_VALUE);
    pwm_set_chan_level(right_pwm_slice, pwm_gpio_to_channel(MOTOR_RIGHT_PWM_PIN), 0);
    pwm_set_enabled(right_pwm_slice, true);
    
    return true;
}

/**
 * @brief Shutdown motor control
 */
void motor_shutdown(void) {
    motor_stop();
    pwm_set_enabled(left_pwm_slice, false);
    pwm_set_enabled(right_pwm_slice, false);
}

/**
 * @brief Enable/disable PID control
 */
void motor_pid_enable(bool enable) {
    left_motor.pid.enabled = enable;
    right_motor.pid.enabled = enable;
    
    if (!enable) {
        motor_reset_pid();
    }
}

/**
 * @brief Set PID parameters
 */
void motor_set_pid_params(float kp, float ki, float kd) {
    left_motor.pid.kp = kp;
    left_motor.pid.ki = ki;
    left_motor.pid.kd = kd;
    
    right_motor.pid.kp = kp;
    right_motor.pid.ki = ki;
    right_motor.pid.kd = kd;
}

/**
 * @brief Compute PID output
 */
static float compute_pid(pid_controller_t* pid, float current_value) {
    float error = pid->setpoint - current_value;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error;
    if (pid->integral > PID_INTEGRAL_MAX) {
        pid->integral = PID_INTEGRAL_MAX;
    } else if (pid->integral < -PID_INTEGRAL_MAX) {
        pid->integral = -PID_INTEGRAL_MAX;
    }
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = error - pid->last_error;
    float d_term = pid->kd * derivative;
    pid->last_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    output = constrain_float(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    
    pid->output = output;
    return output;
}

/**
 * @brief Update PID controllers
 */
void motor_update(void) {
    encoder_update_speed();
    
    left_motor.current_rpm = encoder_get_speed_rpm(true);
    right_motor.current_rpm = encoder_get_speed_rpm(false);
    
    if (left_motor.pid.enabled && left_motor.enabled) {
        left_motor.pid.setpoint = left_motor.target_rpm;
        float left_correction = compute_pid(&left_motor.pid, left_motor.current_rpm);
        
        float new_pwm = left_motor.current_pwm + left_correction;
        new_pwm = constrain_float(new_pwm, MIN_PWM_DUTY, MAX_PWM_DUTY);
        
        set_motor_pwm_duty(true, new_pwm);
        left_motor.current_pwm = new_pwm;
    }
    
    if (right_motor.pid.enabled && right_motor.enabled) {
        right_motor.pid.setpoint = right_motor.target_rpm;
        float right_correction = compute_pid(&right_motor.pid, right_motor.current_rpm);
        
        float new_pwm = right_motor.current_pwm + right_correction;
        new_pwm = constrain_float(new_pwm, MIN_PWM_DUTY, MAX_PWM_DUTY);
        
        set_motor_pwm_duty(false, new_pwm);
        right_motor.current_pwm = new_pwm;
    }
}

/**
 * @brief Set target speed in RPM
 */
void motor_set_speed_rpm(float rpm_left, float rpm_right) {
    left_motor.target_rpm = fabs(rpm_left);
    right_motor.target_rpm = fabs(rpm_right);
    
    left_motor.pid.setpoint = left_motor.target_rpm;
    right_motor.pid.setpoint = right_motor.target_rpm;
    
    // Set initial PWM to get motors moving
    set_motor_pwm_duty(true, 50.0f);  // Start at 50%
    set_motor_pwm_duty(false, 50.0f);
    
    // Set direction based on sign
    if (rpm_left >= 0) {
        set_motor_direction(true, MOTOR_FORWARD);
    } else {
        set_motor_direction(true, MOTOR_BACKWARD);
    }
    
    if (rpm_right >= 0) {
        set_motor_direction(false, MOTOR_FORWARD);
    } else {
        set_motor_direction(false, MOTOR_BACKWARD);
    }
}

/**
 * @brief Set motor direction (Robo PICO: 1=forward, 0=backward)
 */
static void set_motor_direction(bool is_left, motor_direction_t direction) {
    uint8_t dir_pin = is_left ? MOTOR_LEFT_DIR_PIN : MOTOR_RIGHT_DIR_PIN;
    
    switch (direction) {
        case MOTOR_FORWARD:
            gpio_put(dir_pin, 1);
            encoder_set_direction(is_left, true);
            break;
            
        case MOTOR_BACKWARD:
            gpio_put(dir_pin, 0);
            encoder_set_direction(is_left, false);
            break;
            
        case MOTOR_STOP:
            // Keep direction, just set PWM to 0
            break;
    }
    
    if (is_left) {
        left_motor.direction = direction;
    } else {
        right_motor.direction = direction;
    }
}

/**
 * @brief Set PWM duty cycle
 */
static void set_motor_pwm_duty(bool is_left, float duty) {
    duty = constrain_float(duty, 0.0f, MAX_PWM_DUTY);
    
    uint slice = is_left ? left_pwm_slice : right_pwm_slice;
    uint channel = is_left ? 
                   pwm_gpio_to_channel(MOTOR_LEFT_PWM_PIN) : 
                   pwm_gpio_to_channel(MOTOR_RIGHT_PWM_PIN);
    
    uint16_t level = (uint16_t)((duty / 100.0f) * PWM_WRAP_VALUE);
    pwm_set_chan_level(slice, channel, level);
    
    if (is_left) {
        left_motor.current_pwm = duty;
    } else {
        right_motor.current_pwm = duty;
    }
}

/**
 * @brief Set motor PWM directly
 */
void motor_set_pwm(float pwm_left, float pwm_right) {
    set_motor_pwm_duty(true, pwm_left);
    set_motor_pwm_duty(false, pwm_right);
}

/**
 * @brief Move forward
 */
void motor_forward(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    
    set_motor_direction(true, MOTOR_FORWARD);
    set_motor_direction(false, MOTOR_FORWARD);
    
    set_motor_pwm_duty(true, speed);
    set_motor_pwm_duty(false, speed);
}

/**
 * @brief Move backward
 */
void motor_backward(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    
    set_motor_direction(true, MOTOR_BACKWARD);
    set_motor_direction(false, MOTOR_BACKWARD);
    
    set_motor_pwm_duty(true, speed);
    set_motor_pwm_duty(false, speed);
}

/**
 * @brief Turn left
 */
void motor_turn_left(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    float reduced_speed = speed * TURN_SPEED_FACTOR;
    
    set_motor_direction(true, MOTOR_FORWARD);
    set_motor_direction(false, MOTOR_FORWARD);
    
    set_motor_pwm_duty(true, reduced_speed);
    set_motor_pwm_duty(false, speed);
}

/**
 * @brief Turn right
 */
void motor_turn_right(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    float reduced_speed = speed * TURN_SPEED_FACTOR;
    
    set_motor_direction(true, MOTOR_FORWARD);
    set_motor_direction(false, MOTOR_FORWARD);
    
    set_motor_pwm_duty(true, speed);
    set_motor_pwm_duty(false, reduced_speed);
}

/**
 * @brief Pivot turn
 */
void motor_pivot_turn(float speed, bool clockwise) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    
    if (clockwise) {
        set_motor_direction(true, MOTOR_FORWARD);
        set_motor_direction(false, MOTOR_BACKWARD);
    } else {
        set_motor_direction(true, MOTOR_BACKWARD);
        set_motor_direction(false, MOTOR_FORWARD);
    }
    
    set_motor_pwm_duty(true, speed);
    set_motor_pwm_duty(false, speed);
}

/**
 * @brief Stop both motors
 */
void motor_stop(void) {
    set_motor_pwm_duty(true, 0.0f);
    set_motor_pwm_duty(false, 0.0f);
    
    left_motor.target_rpm = 0.0f;
    right_motor.target_rpm = 0.0f;
}

/**
 * @brief Emergency stop
 */
void motor_emergency_stop(void) {
    set_motor_pwm_duty(true, 0.0f);
    set_motor_pwm_duty(false, 0.0f);
    
    motor_stop();
}

/**
 * @brief Set individual motor
 */
void motor_set(bool is_left, motor_direction_t direction, float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    
    set_motor_direction(is_left, direction);
    set_motor_pwm_duty(is_left, speed);
}

/**
 * @brief Adjust motor balance
 */
void motor_adjust_balance(float correction) {
    correction = constrain_float(correction, -50.0f, 50.0f);
    
    if (correction > 0) {
        float new_right = right_motor.current_pwm + correction;
        new_right = constrain_float(new_right, MIN_PWM_DUTY, MAX_PWM_DUTY);
        set_motor_pwm_duty(false, new_right);
    } else if (correction < 0) {
        float new_left = left_motor.current_pwm + fabs(correction);
        new_left = constrain_float(new_left, MIN_PWM_DUTY, MAX_PWM_DUTY);
        set_motor_pwm_duty(true, new_left);
    }
}

/**
 * @brief Get motor state
 */
motor_state_t* motor_get_state(bool is_left) {
    return is_left ? &left_motor : &right_motor;
}

/**
 * @brief Ramp speed smoothly
 */
void motor_ramp_speed(float target_left, float target_right, uint32_t ramp_time_ms) {
    float start_left = left_motor.current_pwm;
    float start_right = right_motor.current_pwm;
    
    float delta_left = target_left - start_left;
    float delta_right = target_right - start_right;
    
    uint32_t steps = ramp_time_ms / 20;
    
    for (uint32_t i = 0; i <= steps; i++) {
        float progress = (float)i / steps;
        
        float current_left = start_left + (delta_left * progress);
        float current_right = start_right + (delta_right * progress);
        
        set_motor_pwm_duty(true, current_left);
        set_motor_pwm_duty(false, current_right);
        
        sleep_ms(20);
    }
}

/**
 * @brief Reset PID controllers
 */
void motor_reset_pid(void) {
    left_motor.pid.integral = 0.0f;
    left_motor.pid.last_error = 0.0f;
    left_motor.pid.output = 0.0f;
    
    right_motor.pid.integral = 0.0f;
    right_motor.pid.last_error = 0.0f;
    right_motor.pid.output = 0.0f;
}

/**
 * @brief Get current PWM
 */
float motor_get_pwm(bool is_left) {
    return is_left ? left_motor.current_pwm : right_motor.current_pwm;
}

/**
 * @brief Constrain float value
 */
static float constrain_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Test motor sequence
 */
void motor_test_sequence(void) {
    printf("=== Motor Test Sequence ===\n");
    
    printf("Test 1: Forward 50%% for 2s\n");
    motor_forward(50);
    sleep_ms(2000);
    motor_stop();
    sleep_ms(1000);
    
    printf("Test 2: Backward 50%% for 2s\n");
    motor_backward(50);
    sleep_ms(2000);
    motor_stop();
    sleep_ms(1000);
    
    printf("Test 3: Turn left for 2s\n");
    motor_turn_left(40);
    sleep_ms(2000);
    motor_stop();
    sleep_ms(1000);
    
    printf("Test 4: Turn right for 2s\n");
    motor_turn_right(40);
    sleep_ms(2000);
    motor_stop();
    sleep_ms(1000);
    
    printf("Test 5: Pivot turn clockwise for 2s\n");
    motor_pivot_turn(35, true);
    sleep_ms(2000);
    motor_stop();
    
    printf("Test complete!\n");
}