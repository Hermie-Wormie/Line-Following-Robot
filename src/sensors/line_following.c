#include "sensors/line_following.h"
#include "sensors/ir_sensor.h"
#include "common/robot_config.h"
#include <stdio.h>

// Tracking variables
static uint32_t line_lost_timestamp = 0;

// Signal strength thresholds
#define SIGNAL_WEAK_THRESHOLD 0.2f    // 20%
#define SIGNAL_STRONG_THRESHOLD 0.6f  // 60%

void line_following_init(void) {
    // Initialize IR sensors (if not already done)
    ir_sensor_init();
    line_lost_timestamp = 0;
    printf("Line following system initialized\n");
}

float get_line_signal_strength(void) {
    uint16_t sensor_value = read_line_sensor();
    return get_signal_strength(sensor_value) / 100.0f;  // Convert to 0.0-1.0
}

line_signal_level_t strength_to_level(float strength) {
    if (strength < SIGNAL_WEAK_THRESHOLD) {
        return SIGNAL_NONE;
    } else if (strength < SIGNAL_STRONG_THRESHOLD) {
        return SIGNAL_WEAK;
    } else {
        return SIGNAL_STRONG;
    }
}

line_signal_level_t get_signal_level(void) {
    float strength = get_line_signal_strength();
    return strength_to_level(strength);
}

bool is_line_detected(void) {
    return get_signal_level() != SIGNAL_NONE;
}

line_state_t get_line_state(void) {
    line_signal_level_t level = get_signal_level();
    
    switch (level) {
        case SIGNAL_STRONG:
            return LINE_DETECTED;
        case SIGNAL_WEAK:
            return LINE_EDGE;
        case SIGNAL_NONE:
        default:
            return LINE_LOST;
    }
}

uint32_t get_line_lost_time_us(void) {
    if (is_line_detected()) {
        line_lost_timestamp = 0;  // Reset timer when line detected
        return 0;
    } else {
        if (line_lost_timestamp == 0) {
            line_lost_timestamp = time_us_32();  // Mark when line was lost
        }
        return time_us_32() - line_lost_timestamp;
    }
}

line_data_t get_line_data(void) {
    line_data_t data;
    
    data.raw_value = read_line_sensor();
    data.signal_strength = get_line_signal_strength();
    data.level = strength_to_level(data.signal_strength);
    data.state = get_line_state();
    data.detected = (data.level != SIGNAL_NONE);
    data.lost_time_us = get_line_lost_time_us();
    
    return data;
}

const char* line_state_to_string(line_state_t state) {
    switch (state) {
        case LINE_DETECTED:  return "DETECTED";
        case LINE_EDGE:      return "EDGE";
        case LINE_LOST:      return "LOST";
        default:             return "UNKNOWN";
    }
}

const char* signal_level_to_string(line_signal_level_t level) {
    switch (level) {
        case SIGNAL_NONE:   return "NONE";
        case SIGNAL_WEAK:   return "WEAK";
        case SIGNAL_STRONG: return "STRONG";
        default:            return "UNKNOWN";
    }
}