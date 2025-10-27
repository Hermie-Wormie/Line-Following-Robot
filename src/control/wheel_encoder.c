#include "control/wheel_encoder.h"
#include "common/robot_config.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h>

// Encoder state
typedef struct {
    uint pin_a;
    uint pin_b;
    volatile int32_t count;
    uint32_t last_pulse_time;
    uint32_t pulse_period;
    bool initialized;
} encoder_state_t;

static encoder_state_t encoders[2];
static bool system_initialized = false;

// Constants for calculations
static const float MM_PER_PULSE = WHEEL_CIRCUMFERENCE_MM / ENCODER_PULSES_PER_REV;
static const float SPEED_TIMEOUT_US = 500000;  // 500ms - consider stopped if no pulse

/**
 * @brief Interrupt handler for left encoder (Motor 1)
 */
static void encoder_left_isr(uint gpio, uint32_t events) {
    encoder_state_t *enc = &encoders[ENCODER_LEFT];
    
    bool a = gpio_get(enc->pin_a);
    bool b = gpio_get(enc->pin_b);
    
    // Determine direction based on A and B phases
    if (gpio == enc->pin_a) {
        if (a == b) {
            enc->count++;  // Forward
        } else {
            enc->count--;  // Backward
        }
    }
    
    // Update timing for speed calculation
    uint32_t current_time = time_us_32();
    enc->pulse_period = current_time - enc->last_pulse_time;
    enc->last_pulse_time = current_time;
}

/**
 * @brief Interrupt handler for right encoder (Motor 2)
 */
static void encoder_right_isr(uint gpio, uint32_t events) {
    encoder_state_t *enc = &encoders[ENCODER_RIGHT];
    
    bool a = gpio_get(enc->pin_a);
    bool b = gpio_get(enc->pin_b);
    
    // Determine direction based on A and B phases
    if (gpio == enc->pin_a) {
        if (a == b) {
            enc->count++;  // Forward
        } else {
            enc->count--;  // Backward
        }
    }
    
    // Update timing for speed calculation
    uint32_t current_time = time_us_32();
    enc->pulse_period = current_time - enc->last_pulse_time;
    enc->last_pulse_time = current_time;
}

/**
 * @brief Initialize a single encoder
 */
static void encoder_init_single(encoder_state_t *enc, uint pin_a, uint pin_b, 
                                 gpio_irq_callback_t callback) {
    enc->pin_a = pin_a;
    enc->pin_b = pin_b;
    enc->count = 0;
    enc->last_pulse_time = 0;
    enc->pulse_period = 0;
    
    // Initialize GPIO pins
    gpio_init(pin_a);
    gpio_init(pin_b);
    gpio_set_dir(pin_a, GPIO_IN);
    gpio_set_dir(pin_b, GPIO_IN);
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_b);
    
    // Setup interrupt on pin A (rising and falling edges)
    gpio_set_irq_enabled_with_callback(pin_a, 
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, 
                                       callback);
    
    enc->initialized = true;
}

void encoder_init(void) {
    if (system_initialized) {
        return;  // Already initialized
    }
    
    // Initialize left encoder (Motor 1)
    encoder_init_single(&encoders[ENCODER_LEFT], 
                       MOTOR1_ENCODER_A_PIN, 
                       MOTOR1_ENCODER_B_PIN,
                       encoder_left_isr);
    
    // Initialize right encoder (Motor 2)
    encoder_init_single(&encoders[ENCODER_RIGHT],
                       MOTOR2_ENCODER_A_PIN,
                       MOTOR2_ENCODER_B_PIN,
                       encoder_right_isr);
    
    system_initialized = true;
    
    printf("Wheel encoders initialized\n");
    printf("  Left Encoder:  A=GP%d, B=GP%d\n", MOTOR1_ENCODER_A_PIN, MOTOR1_ENCODER_B_PIN);
    printf("  Right Encoder: A=GP%d, B=GP%d\n", MOTOR2_ENCODER_A_PIN, MOTOR2_ENCODER_B_PIN);
    printf("  Resolution: %.2f mm/pulse\n", MM_PER_PULSE);
}

void encoder_reset(encoder_id_t encoder) {
    if (!system_initialized || encoder > ENCODER_RIGHT) {
        return;
    }
    
    encoders[encoder].count = 0;
    encoders[encoder].last_pulse_time = 0;
    encoders[encoder].pulse_period = 0;
}

void encoder_reset_all(void) {
    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);
}

int32_t encoder_get_count(encoder_id_t encoder) {
    if (!system_initialized || encoder > ENCODER_RIGHT) {
        return 0;
    }
    return encoders[encoder].count;
}

float encoder_get_distance_mm(encoder_id_t encoder) {
    if (!system_initialized || encoder > ENCODER_RIGHT) {
        return 0.0f;
    }
    
    int32_t count = encoders[encoder].count;
    return (float)count * MM_PER_PULSE;
}

float encoder_get_speed_rpm(encoder_id_t encoder) {
    if (!system_initialized || encoder > ENCODER_RIGHT) {
        return 0.0f;
    }
    
    encoder_state_t *enc = &encoders[encoder];
    
    // Check if we've had a recent pulse
    uint32_t time_since_last_pulse = time_us_32() - enc->last_pulse_time;
    if (time_since_last_pulse > SPEED_TIMEOUT_US) {
        return 0.0f;  // No recent pulse, assume stopped
    }
    
    if (enc->pulse_period == 0) {
        return 0.0f;  // No data yet
    }
    
    // Calculate RPM from pulse period
    // RPM = (60,000,000 / pulse_period_us) / pulses_per_rev
    float pulses_per_second = 1000000.0f / enc->pulse_period;
    float rps = pulses_per_second / ENCODER_PULSES_PER_REV;
    float rpm = rps * 60.0f;
    
    return rpm;
}

float encoder_get_speed_mmps(encoder_id_t encoder) {
    float rpm = encoder_get_speed_rpm(encoder);
    
    // Convert RPM to mm/s
    // mm/s = (RPM * circumference) / 60
    return (rpm * WHEEL_CIRCUMFERENCE_MM) / 60.0f;
}

encoder_data_t encoder_get_data(encoder_id_t encoder) {
    encoder_data_t data = {0};
    
    if (!system_initialized || encoder > ENCODER_RIGHT) {
        return data;
    }
    
    data.count = encoder_get_count(encoder);
    data.distance_mm = encoder_get_distance_mm(encoder);
    data.speed_rpm = encoder_get_speed_rpm(encoder);
    data.last_update = encoders[encoder].last_pulse_time;
    data.pulse_period = encoders[encoder].pulse_period;
    
    return data;
}

void encoder_update_speed(void) {
    // This function can be called periodically to check for timeout
    // Speed calculation is done on-demand in encoder_get_speed_rpm()
    // But we can use this to reset stale data
    
    uint32_t current_time = time_us_32();
    
    for (int i = 0; i < 2; i++) {
        uint32_t time_since_pulse = current_time - encoders[i].last_pulse_time;
        if (time_since_pulse > SPEED_TIMEOUT_US) {
            // No pulse for a while, reset pulse period
            encoders[i].pulse_period = 0;
        }
    }
}

bool encoder_is_initialized(void) {
    return system_initialized;
}

float encoder_get_average_speed_mmps(void) {
    float left_speed = encoder_get_speed_mmps(ENCODER_LEFT);
    float right_speed = encoder_get_speed_mmps(ENCODER_RIGHT);
    return (left_speed + right_speed) / 2.0f;
}

float encoder_get_average_distance_mm(void) {
    float left_dist = encoder_get_distance_mm(ENCODER_LEFT);
    float right_dist = encoder_get_distance_mm(ENCODER_RIGHT);
    return (left_dist + right_dist) / 2.0f;
}