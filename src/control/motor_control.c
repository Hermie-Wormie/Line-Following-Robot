#include "control/motor_control.h"
#include "common/robot_config.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>

// Motor state tracking
typedef struct {
    uint pwm_pin;
    uint dir_pin;
    uint slice;
    uint channel;
    int current_speed;  // -100 to 100
    bool initialized;
} motor_state_t;

static motor_state_t motors[2];
static bool system_initialized = false;

/**
 * @brief Initialize a single motor
 */
static void motor_init_single(motor_state_t *motor, uint pwm_pin, uint dir_pin) {
    motor->pwm_pin = pwm_pin;
    motor->dir_pin = dir_pin;
    motor->current_speed = 0;
    
    // Initialize direction pin
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 0);
    
    // Initialize PWM pin
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    motor->slice = pwm_gpio_to_slice_num(pwm_pin);
    motor->channel = pwm_gpio_to_channel(pwm_pin);
    
    // Configure PWM
    pwm_config config = pwm_get_default_config();
    
    // Set frequency to MOTOR_PWM_FREQUENCY (default 1000 Hz)
    // System clock is 125 MHz
    float clkdiv = 125000000.0f / (MOTOR_PWM_FREQUENCY * 65535);
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_config_set_wrap(&config, 65535);
    
    pwm_init(motor->slice, &config, true);
    pwm_set_chan_level(motor->slice, motor->channel, 0);
    
    motor->initialized = true;
}

/**
 * @brief Set PWM duty cycle for a motor
 * @param motor Motor state structure
 * @param duty_cycle Duty cycle (0-100%)
 */
static void motor_set_pwm(motor_state_t *motor, uint duty_cycle) {
    if (!motor->initialized) return;
    
    // Clamp duty cycle
    if (duty_cycle > 100) duty_cycle = 100;
    
    // Convert percentage to PWM level (0-65535)
    uint16_t level = (uint16_t)((duty_cycle * 65535) / 100);
    pwm_set_chan_level(motor->slice, motor->channel, level);
}

/**
 * @brief Set motor direction
 * @param motor Motor state structure
 * @param direction MOTOR_FORWARD or MOTOR_BACKWARD
 */
static void motor_set_direction(motor_state_t *motor, motor_direction_t direction) {
    if (!motor->initialized) return;
    gpio_put(motor->dir_pin, direction);
}

void motor_control_init(void) {
    if (system_initialized) {
        return;  // Already initialized
    }
    
    // Initialize left motor (Motor 1)
    motor_init_single(&motors[MOTOR_LEFT], MOTOR1_PWM_PIN, MOTOR1_DIR_PIN);
    
    // Initialize right motor (Motor 2)
    motor_init_single(&motors[MOTOR_RIGHT], MOTOR2_PWM_PIN, MOTOR2_DIR_PIN);
    
    system_initialized = true;
    
    printf("Motor control initialized\n");
    printf("  Left Motor:  PWM=GP%d, DIR=GP%d\n", MOTOR1_PWM_PIN, MOTOR1_DIR_PIN);
    printf("  Right Motor: PWM=GP%d, DIR=GP%d\n", MOTOR2_PWM_PIN, MOTOR2_DIR_PIN);
    printf("  PWM Frequency: %d Hz\n", MOTOR_PWM_FREQUENCY);
}

void motor_set_speed(motor_id_t motor, int speed) {
    if (!system_initialized || motor > MOTOR_RIGHT) {
        return;
    }
    
    motor_state_t *m = &motors[motor];
    
    // Clamp speed to valid range
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    m->current_speed = speed;
    
    // Set direction based on sign
    if (speed >= 0) {
        motor_set_direction(m, MOTOR_FORWARD);
        motor_set_pwm(m, abs(speed));
    } else {
        motor_set_direction(m, MOTOR_BACKWARD);
        motor_set_pwm(m, abs(speed));
    }
}

void motor_stop(motor_id_t motor) {
    if (!system_initialized || motor > MOTOR_RIGHT) {
        return;
    }
    
    motor_set_speed(motor, 0);
}

void motor_stop_all(void) {
    motor_stop(MOTOR_LEFT);
    motor_stop(MOTOR_RIGHT);
}

void motor_forward(int speed) {
    motor_set_speed(MOTOR_LEFT, speed);
    motor_set_speed(MOTOR_RIGHT, speed);
}

void motor_backward(int speed) {
    motor_set_speed(MOTOR_LEFT, -speed);
    motor_set_speed(MOTOR_RIGHT, -speed);
}

void motor_turn_left(int speed) {
    // Left motor at half speed, right motor at full speed
    motor_set_speed(MOTOR_LEFT, speed / 2);
    motor_set_speed(MOTOR_RIGHT, speed);
}

void motor_turn_right(int speed) {
    // Right motor at half speed, left motor at full speed
    motor_set_speed(MOTOR_LEFT, speed);
    motor_set_speed(MOTOR_RIGHT, speed / 2);
}

void motor_rotate_left(int speed) {
    // Left motor backward, right motor forward (rotate in place)
    motor_set_speed(MOTOR_LEFT, -speed);
    motor_set_speed(MOTOR_RIGHT, speed);
}

void motor_rotate_right(int speed) {
    // Left motor forward, right motor backward (rotate in place)
    motor_set_speed(MOTOR_LEFT, speed);
    motor_set_speed(MOTOR_RIGHT, -speed);
}

void motor_set_differential(int left_speed, int right_speed) {
    motor_set_speed(MOTOR_LEFT, left_speed);
    motor_set_speed(MOTOR_RIGHT, right_speed);
}

int motor_get_speed(motor_id_t motor) {
    if (!system_initialized || motor > MOTOR_RIGHT) {
        return 0;
    }
    return motors[motor].current_speed;
}

bool motor_is_initialized(void) {
    return system_initialized;
}