#include "sensors/ir_sensor.h"
#include "common/robot_config.h"
#include <stdio.h>

void ir_sensor_init(void) {
    // Initialize ADC hardware
    adc_init();
    
    // Configure GPIO pins for ADC
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_gpio_init(BARCODE_SENSOR_PIN);
    
    printf("IR sensors initialized - Line: GP%d, Barcode: GP%d\n", 
           LINE_SENSOR_PIN, BARCODE_SENSOR_PIN);
}

uint16_t read_line_sensor_raw(void) {
    adc_select_input(LINE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

uint16_t read_barcode_sensor_raw(void) {
    adc_select_input(BARCODE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

uint16_t read_sensor_averaged(uint8_t adc_channel) {
    adc_select_input(adc_channel);
    uint32_t sum = 0;
    
    // Take multiple samples for stability
    for (int i = 0; i < SENSOR_SAMPLE_COUNT; i++) {
        sum += adc_read();
        sleep_us(SENSOR_SAMPLE_DELAY_US);
    }
    
    return sum / SENSOR_SAMPLE_COUNT;
}

uint16_t read_line_sensor(void) {
    return read_sensor_averaged(LINE_SENSOR_ADC_CHANNEL);
}

uint16_t read_barcode_sensor(void) {
    return read_sensor_averaged(BARCODE_SENSOR_ADC_CHANNEL);
}

bool is_black_detected(uint16_t sensor_value) {
    return sensor_value > THRESHOLD;
}

float get_signal_strength(uint16_t sensor_value) {
    // Clamp values to expected range
    if (sensor_value < WHITE_VALUE) return 0.0f;
    if (sensor_value > BLACK_VALUE) return 100.0f;
    
    // Calculate percentage
    return (float)(sensor_value - WHITE_VALUE) / (BLACK_VALUE - WHITE_VALUE) * 100.0f;
}