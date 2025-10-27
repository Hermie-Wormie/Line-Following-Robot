#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

/**
 * @brief Initialize IR sensors
 * Sets up ADC and configures both line and barcode sensor pins
 */
void ir_sensor_init(void);

/**
 * @brief Read raw value from line following sensor
 * @return ADC reading (0-4095)
 */
uint16_t read_line_sensor_raw(void);

/**
 * @brief Read raw value from barcode scanning sensor  
 * @return ADC reading (0-4095)
 */
uint16_t read_barcode_sensor_raw(void);

/**
 * @brief Read line sensor with averaging for stability
 * @return Averaged ADC reading
 */
uint16_t read_line_sensor(void);

/**
 * @brief Read barcode sensor with averaging for stability
 * @return Averaged ADC reading
 */
uint16_t read_barcode_sensor(void);

/**
 * @brief Check if sensor value indicates black surface
 * @param sensor_value ADC reading to check
 * @return true if black detected, false if white
 */
bool is_black_detected(uint16_t sensor_value);

/**
 * @brief Calculate signal strength as percentage
 * @param sensor_value ADC reading
 * @return Signal strength (0.0 = white, 100.0 = black)
 */
float get_signal_strength(uint16_t sensor_value);

/**
 * @brief Read specific ADC channel with averaging
 * @param adc_channel ADC channel (0 or 1)
 * @return Averaged ADC reading
 */
uint16_t read_sensor_averaged(uint8_t adc_channel);

// ============ NEW: DIGITAL PULSE WIDTH FUNCTIONS ============
/**
 * @brief Initialize IR sensor in digital mode for pulse width measurement
 * Demonstrates digital signal processing capability
 * @param digital_pin GPIO pin configured for digital IR output
 */
void ir_sensor_digital_init(uint digital_pin);

/**
 * @brief Get last measured pulse width from digital IR sensor
 * Demonstrates pulse width measurement using digital signals
 * @return Pulse width in microseconds
 */
uint32_t ir_get_pulse_width_us(void);

/**
 * @brief Get total count of digital pulses detected
 * @return Total pulse count
 */
uint32_t ir_get_pulse_count(void);

/**
 * @brief Get frequency of digital pulses
 * @return Frequency in Hz
 */
float ir_get_pulse_frequency_hz(void);

/**
 * @brief Reset digital pulse counters
 */
void ir_reset_pulse_count(void);

/**
 * @brief Check if IR sensor is currently detecting (digital mode)
 * @return true if sensor detects object/line
 */
bool ir_digital_state(void);

#endif