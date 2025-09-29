#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// IR Sensor Pin Configuration
#define LINE_SENSOR_PIN 26      // ADC0 - GP26
#define BARCODE_SENSOR_PIN 27   // ADC1 - GP27

// IR Sensor Calibrated Values (update with your measurements)
#define WHITE_VALUE 210
#define BLACK_VALUE 3000
#define THRESHOLD 1605          // (WHITE_VALUE + BLACK_VALUE) / 2

// ADC Configuration
#define LINE_SENSOR_ADC_CHANNEL 0    // GP26 = ADC0
#define BARCODE_SENSOR_ADC_CHANNEL 1 // GP27 = ADC1

// Sensor Reading Configuration
#define SENSOR_SAMPLE_COUNT 5        // Number of samples to average
#define SENSOR_SAMPLE_DELAY_US 100   // Delay between samples

// Line Following Configuration
#define LINE_POSITION_SCALE 2.0f     // Scale factor for line position (-1.0 to +1.0)

// Barcode Configuration
#define MAX_BARCODE_ELEMENTS 100     // Maximum barcode elements to capture
#define BARCODE_TRANSITION_THRESHOLD 500  // ADC difference for transitions
#define BARCODE_END_TIMEOUT_US 10000 // Timeout for end of barcode (10ms)

// Motor pins (placeholders for teammates)
#define MOTOR_LEFT_PIN1 2
#define MOTOR_LEFT_PIN2 3
#define MOTOR_RIGHT_PIN1 4
#define MOTOR_RIGHT_PIN2 5

// Other sensor pins (placeholders for teammates)
#define ULTRASONIC_TRIG_PIN 10
#define ULTRASONIC_ECHO_PIN 11
#define IMU_SDA_PIN 12
#define IMU_SCL_PIN 13

#endif