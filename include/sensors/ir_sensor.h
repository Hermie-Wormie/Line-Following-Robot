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

#endif