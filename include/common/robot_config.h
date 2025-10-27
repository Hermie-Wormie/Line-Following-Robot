#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/**
 * @file robot_config.h
 * @brief Central configuration file for INF2004 Robotic Car Project
 * @team IS21
 * 
 * All hardware pin assignments and calibration values are defined here.
 * Update this file when making hardware changes.
 */

// ============================================================================
// MOTOR CONTROL PINS (Built-in Motor Ports)
// ============================================================================
#define MOTOR1_PWM_PIN  8   // Motor 1 PWM (Left motor)
#define MOTOR1_DIR_PIN  9   // Motor 1 Direction
#define MOTOR2_PWM_PIN  10  // Motor 2 PWM (Right motor)
#define MOTOR2_DIR_PIN  11  // Motor 2 Direction

// ============================================================================
// WHEEL ENCODER PINS
// ============================================================================
// Motor 1 Encoder - Grove 5
#define MOTOR1_ENCODER_A_PIN  6   // GP6
#define MOTOR1_ENCODER_B_PIN  26  // GP26

// Motor 2 Encoder - Grove 2
#define MOTOR2_ENCODER_A_PIN  2   // GP2
#define MOTOR2_ENCODER_B_PIN  3   // GP3

// Encoder Configuration
#define ENCODER_PULSES_PER_REV  20    // Pulses per revolution (adjust for your encoder)
#define WHEEL_DIAMETER_MM       65.0f  // Wheel diameter in mm
#define WHEEL_CIRCUMFERENCE_MM  204.2f // π * diameter

// ============================================================================
// IR SENSOR PINS (Analog)
// ============================================================================
#define LINE_SENSOR_PIN     27  // ADC1 - GP27 (Line following)
#define BARCODE_SENSOR_PIN  26  // ADC0 - GP26 (Barcode reading)

// IR Sensor ADC Channels
#define LINE_SENSOR_ADC_CHANNEL     1  // GP27 = ADC1
#define BARCODE_SENSOR_ADC_CHANNEL  0  // GP26 = ADC0

// IR Sensor Calibrated Values (IMPORTANT: Calibrate these for your setup!)
#define WHITE_VALUE  210    // ADC reading on white surface
#define BLACK_VALUE  3000   // ADC reading on black surface
#define THRESHOLD    1605   // (WHITE_VALUE + BLACK_VALUE) / 2

// Sensor Reading Configuration
#define SENSOR_SAMPLE_COUNT     5    // Number of samples to average
#define SENSOR_SAMPLE_DELAY_US  100  // Delay between samples in microseconds

// ============================================================================
// BARCODE SCANNER CONFIGURATION
// ============================================================================
#define MAX_BARCODE_ELEMENTS           100    // Maximum barcode elements to capture
#define BARCODE_TRANSITION_THRESHOLD   500    // ADC difference for transitions
#define BARCODE_END_TIMEOUT_US         10000  // Timeout for end of barcode (10ms)

// ============================================================================
// ULTRASONIC SENSOR PINS
// ============================================================================
#define ULTRASONIC_TRIG_PIN  3   // GP3 - Trigger pin
#define ULTRASONIC_ECHO_PIN  2   // GP2 - Echo pin
#define SERVO_PIN            15  // GP15 - Servo for ultrasonic pan

// Ultrasonic Configuration
#define OBSTACLE_DETECTION_THRESHOLD_MM  100  // Distance to consider as obstacle
#define ULTRASONIC_MIN_QUIET_TIME_US     60000 // 60ms between measurements

// ============================================================================
// PID CONTROLLER CONFIGURATION
// ============================================================================
// Line Following PID (tune these values during testing!)
#define LINE_PID_KP  2.0f   // Proportional gain
#define LINE_PID_KI  0.0f   // Integral gain
#define LINE_PID_KD  0.5f   // Derivative gain

// Motor Speed PID (tune these values during testing!)
#define MOTOR_PID_KP  1.0f
#define MOTOR_PID_KI  0.1f
#define MOTOR_PID_KD  0.05f

// ============================================================================
// MOTOR CONTROL CONFIGURATION
// ============================================================================
#define MOTOR_BASE_SPEED     50   // Base speed (0-100%)
#define MOTOR_MAX_SPEED      100  // Maximum speed (0-100%)
#define MOTOR_MIN_SPEED      20   // Minimum speed (0-100%)
#define MOTOR_PWM_FREQUENCY  1000 // PWM frequency in Hz

// ============================================================================
// IMU SENSOR PINS (I2C) - MPU9250
// ============================================================================
#define IMU_SDA_PIN  4   // I2C0 SDA
#define IMU_SCL_PIN  5   // I2C0 SCL
#define IMU_I2C_PORT i2c0
#define IMU_I2C_FREQ 400000  // 400kHz

// ============================================================================
// WiFi / MQTT CONFIGURATION (Pico W)
// ============================================================================
// WiFi credentials (for testing - move to secure storage in production)
#define WIFI_SSID     "your_ssid"
#define WIFI_PASSWORD "your_password"

// MQTT Broker settings
#define MQTT_BROKER_IP   "192.168.1.100"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "robotic_car_is21"

// MQTT Topics
#define MQTT_TOPIC_TELEMETRY  "robot/telemetry"
#define MQTT_TOPIC_COMMAND    "robot/command"
#define MQTT_TOPIC_STATUS     "robot/status"

// ============================================================================
// SYSTEM TIMING CONFIGURATION
// ============================================================================
#define MAIN_LOOP_DELAY_MS        50    // Main control loop delay
#define SENSOR_UPDATE_RATE_MS     20    // How often to read sensors
#define TELEMETRY_UPDATE_RATE_MS  500   // How often to send telemetry
#define BARCODE_COOLDOWN_MS       2000  // Time after barcode before scanning again

// ============================================================================
// DEBUG AND TESTING FLAGS
// ============================================================================
#define DEBUG_MOTOR_CONTROL   0  // Enable motor debug output
#define DEBUG_LINE_FOLLOWING  0  // Enable line following debug output
#define DEBUG_BARCODE_SCANNER 0  // Enable barcode debug output
#define DEBUG_PID_CONTROLLER  0  // Enable PID debug output
#define DEBUG_ENCODER         0  // Enable encoder debug output

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================
#define ROBOT_WHEELBASE_MM    150.0f  // Distance between left and right wheels
#define ROBOT_TRACK_WIDTH_MM  120.0f  // Width of the robot

#endif // ROBOT_CONFIG_H