/**
 * @file motor_control.c
 * @brief Motor Control Implementation for Robo PICO - STANDALONE VERSION
 * Works WITHOUT encoders - safe for simple motor testing
 */

#include "motor_control.h"
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
static float constrain_float(float value, float min, float max);

/**
 * @brief Constrain float value
 */
static float constrain_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

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
    left_motor.pid.enabled = false;
    
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
    right_motor.pid.enabled = false;
    
    // Initialize direction pins
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
 * @brief Set motor direction
 * NOTE: encoder_set_direction() calls removed to avoid linking errors
 */
static void set_motor_direction(bool is_left, motor_direction_t direction) {
    uint8_t dir_pin = is_left ? MOTOR_LEFT_DIR_PIN : MOTOR_RIGHT_DIR_PIN;
    
    switch (direction) {
        case MOTOR_FORWARD:
            gpio_put(dir_pin, 1);
            // Encoder notification removed - add back when encoders are integrated
            break;
            
        case MOTOR_BACKWARD:
            gpio_put(dir_pin, 0);
            // Encoder notification removed - add back when encoders are integrated
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
 * @brief Simple speed control - THE MISSING FUNCTION!
 * @param speed_left Left motor speed (-100 to +100, negative = backward)
 * @param speed_right Right motor speed (-100 to +100, negative = backward)
 */
void motor_set_speed(float speed_left, float speed_right) {
    // Set directions based on sign
    if (speed_left >= 0) {
        set_motor_direction(true, MOTOR_FORWARD);
    } else {
        set_motor_direction(true, MOTOR_BACKWARD);
        speed_left = -speed_left;
    }
    
    if (speed_right >= 0) {
        set_motor_direction(false, MOTOR_FORWARD);
    } else {
        set_motor_direction(false, MOTOR_BACKWARD);
        speed_right = -speed_right;
    }
    
    // Constrain to valid range
    speed_left = constrain_float(speed_left, 0.0f, MAX_PWM_DUTY);
    speed_right = constrain_float(speed_right, 0.0f, MAX_PWM_DUTY);
    
    // Set PWM
    set_motor_pwm_duty(true, speed_left);
    set_motor_pwm_duty(false, speed_right);
}

/**
 * @brief Set motor PWM directly (alias for motor_set_speed)
 */
void motor_set_pwm(float pwm_left, float pwm_right) {
    motor_set_speed(pwm_left, pwm_right);
}

/**
 * @brief Move forward at specified speed
 */
void motor_forward(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    motor_set_speed(speed, speed);
}

/**
 * @brief Move backward at specified speed
 */
void motor_backward(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    motor_set_speed(-speed, -speed);
}

/**
 * @brief Turn left (slower left wheel)
 */
void motor_turn_left(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    float reduced_speed = speed * TURN_SPEED_FACTOR;
    motor_set_speed(reduced_speed, speed);
}

/**
 * @brief Turn right (slower right wheel)
 */
void motor_turn_right(float speed) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    float reduced_speed = speed * TURN_SPEED_FACTOR;
    motor_set_speed(speed, reduced_speed);
}

/**
 * @brief Pivot turn in place
 */
void motor_pivot_turn(float speed, bool clockwise) {
    speed = constrain_float(speed, 0.0f, MAX_PWM_DUTY);
    
    if (clockwise) {
        motor_set_speed(speed, -speed);
    } else {
        motor_set_speed(-speed, speed);
    }
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
 * @brief Get current PWM duty cycle
 */
float motor_get_pwm(bool is_left) {
    return is_left ? left_motor.current_pwm : right_motor.current_pwm;
}

/**
 * @brief Get motor state
 */
motor_state_t* motor_get_state(bool is_left) {
    return is_left ? &left_motor : &right_motor;
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