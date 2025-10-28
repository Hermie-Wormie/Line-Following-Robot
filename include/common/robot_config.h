/**
 * @file robot_config.h
 * @brief Robot Hardware Configuration for Robo PICO
 * 
 * This file contains all hardware pin definitions and configuration
 * for the Robo PICO robot platform
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Motor Control Pins (Robo PICO onboard)
#define MOTOR1_PWM_PIN          8   // GP8 - Left Motor PWM
#define MOTOR1_DIR_PIN          9   // GP9 - Left Motor Direction
#define MOTOR2_PWM_PIN          10  // GP10 - Right Motor PWM
#define MOTOR2_DIR_PIN          11  // GP11 - Right Motor Direction

// Encoder Pins (Grove Connectors)
// Motor 1 (Left) Encoder on Grove 5
#define MOTOR1_ENCODER_A_PIN    6   // GP6 - Grove 5 (Yellow wire)

// Motor 2 (Right) Encoder on Grove 2
#define MOTOR2_ENCODER_A_PIN    2   // GP2 - Grove 2 (Yellow wire)

// Wheel Parameters
#define WHEEL_DIAMETER_MM       65      // 65mm diameter wheels
#define WHEEL_BASE_MM           120     // Distance between wheels (adjust as needed)
#define ENCODER_PULSES_PER_REV  20      // XCH206 has 20 slots

// Motor Parameters
#define DEFAULT_MOTOR_SPEED     50      // Default speed (0-100%)
#define MAX_MOTOR_SPEED         100     // Maximum speed limit
#define MIN_MOTOR_SPEED         15      // Minimum speed to overcome friction

// PID Control Parameters
#define PID_KP                  0.8f    // Proportional gain
#define PID_KI                  0.1f    // Integral gain
#define PID_KD                  0.05f   // Derivative gain
#define PID_UPDATE_RATE_HZ      10      // PID update frequency (10Hz = 100ms)

#endif // ROBOT_CONFIG_H