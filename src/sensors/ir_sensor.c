#include "sensors/ir_sensor.h"
#include "common/robot_config.h"
#include <stdio.h>

void ir_sensor_init(void) {
    adc_init();
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
    if (sensor_value < WHITE_VALUE) return 0.0f;
    if (sensor_value > BLACK_VALUE) return 100.0f;
    return (float)(sensor_value - WHITE_VALUE) / (BLACK_VALUE - WHITE_VALUE) * 100.0f;
}

// ============================================================
// DIGITAL SIGNAL PROCESSING - PULSE WIDTH MEASUREMENT
// Measures duration of BLACK detection (HIGH state)
// ============================================================

static volatile uint32_t digital_pulse_count = 0;
static volatile uint32_t digital_pulse_width_us = 0;
static volatile uint32_t digital_rising_edge_time = 0;  // When HIGH started
static volatile bool digital_current_state = false;

#define MIN_PULSE_WIDTH_US 1000  // Ignore pulses shorter than 1ms

void ir_digital_callback(uint gpio, uint32_t events) {
    (void)gpio;
    uint32_t current_time = time_us_32();
    
    if (events & GPIO_IRQ_EDGE_RISE) {
        // WHITE → BLACK transition (LOW → HIGH)
        digital_rising_edge_time = current_time;  // Record when BLACK started
        digital_current_state = true;
    }
    else if (events & GPIO_IRQ_EDGE_FALL) {
        // BLACK → WHITE transition (HIGH → LOW)
        digital_current_state = false;
        
        if (digital_rising_edge_time > 0) {
            // Calculate how long it stayed BLACK
            uint32_t width = current_time - digital_rising_edge_time;
            
            // Filter noise
            if (width >= MIN_PULSE_WIDTH_US) {
                digital_pulse_width_us = width;  // Store BLACK duration
                digital_pulse_count++;
            }
        }
    }
}

void ir_sensor_digital_init(uint digital_pin) {
    gpio_init(digital_pin);
    gpio_set_dir(digital_pin, GPIO_IN);
    gpio_pull_down(digital_pin);
    
    sleep_ms(10);
    
    gpio_set_irq_enabled_with_callback(
        digital_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &ir_digital_callback
    );
    
    printf("IR Sensor DIGITAL mode initialized on GP%d\n", digital_pin);
    printf("Measures: Duration of BLACK detection (HIGH state)\n");
}

uint32_t ir_get_pulse_width_us(void) {
    return digital_pulse_width_us;
}

uint32_t ir_get_pulse_count(void) {
    return digital_pulse_count;
}

float ir_get_pulse_frequency_hz(void) {
    if (digital_pulse_width_us == 0) {
        return 0.0f;
    }
    return 1000000.0f / (float)digital_pulse_width_us;
}

void ir_reset_pulse_count(void) {
    digital_pulse_count = 0;
    digital_pulse_width_us = 0;
    digital_rising_edge_time = 0;
}

bool ir_digital_state(void) {
    return digital_current_state;
}