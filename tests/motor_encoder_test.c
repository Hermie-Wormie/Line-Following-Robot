#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#include "robot_config.h"

int main() {
    // Initialize USB serial
    stdio_init_all();
    
    // Wait for USB serial to be ready
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("Motor and Encoder Test Starting...\n");
    printf("========================================\n\n");
    
    // Initialize motor controller
    printf("Initializing motor controller...\n");
    motor_init();
    sleep_ms(500);
    
    // Initialize wheel encoders with correct GPIO pins
    // Motor 1 (Left) on Grove 5: GP6
    // Motor 2 (Right) on Grove 2: GP2
    printf("Initializing wheel encoders...\n");
    printf("  Motor 1 encoder on GP6 (Grove 5)\n");
    printf("  Motor 2 encoder on GP2 (Grove 2)\n");
    encoder_init(MOTOR1_ENCODER_A_PIN, MOTOR2_ENCODER_A_PIN);  // GP6, GP2
    sleep_ms(500);
    
    printf("Initialization complete!\n\n");
    
    // Test 1: Forward motion
    printf("TEST 1: Moving forward at 50 RPM for 3 seconds\n");
    motor_set_speed_rpm(50.0f, 50.0f);
    encoder_set_direction(true, true);   // Left motor forward
    encoder_set_direction(false, true);  // Right motor forward
    
    for (int i = 0; i < 30; i++) {
        encoder_update_speed();  // Update speed calculations
        
        float motor1_speed = encoder_get_speed_rpm(true);   // Left (Motor1)
        float motor2_speed = encoder_get_speed_rpm(false);  // Right (Motor2)
        float motor1_dist = encoder_get_distance_mm(true) / 10.0f;   // Convert mm to cm
        float motor2_dist = encoder_get_distance_mm(false) / 10.0f;  // Convert mm to cm
        
        printf("  Motor1 (Left): %.2f RPM, %.2f cm | Motor2 (Right): %.2f RPM, %.2f cm\n", 
               motor1_speed, motor1_dist, motor2_speed, motor2_dist);
        sleep_ms(100);
    }
    
    // Stop
    printf("\nStopping motors...\n");
    motor_stop();
    sleep_ms(1000);
    
    // Test 2: Backward motion
    printf("\nTEST 2: Moving backward at 30 RPM for 2 seconds\n");
    motor_set_speed_rpm(-30.0f, -30.0f);
    encoder_set_direction(true, false);   // Left motor backward
    encoder_set_direction(false, false);  // Right motor backward
    
    for (int i = 0; i < 20; i++) {
        encoder_update_speed();
        
        float motor1_speed = encoder_get_speed_rpm(true);
        float motor2_speed = encoder_get_speed_rpm(false);
        
        printf("  Motor1: %.2f RPM | Motor2: %.2f RPM\n", motor1_speed, motor2_speed);
        sleep_ms(100);
    }
    
    // Stop
    printf("\nStopping motors...\n");
    motor_stop();
    sleep_ms(1000);
    
    // Test 3: Turn left (motor2 faster)
    printf("\nTEST 3: Turning left for 2 seconds\n");
    motor_set_speed_rpm(20.0f, 40.0f);
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    for (int i = 0; i < 20; i++) {
        encoder_update_speed();
        
        float motor1_speed = encoder_get_speed_rpm(true);
        float motor2_speed = encoder_get_speed_rpm(false);
        
        printf("  Motor1: %.2f RPM | Motor2: %.2f RPM\n", motor1_speed, motor2_speed);
        sleep_ms(100);
    }
    
    // Stop
    printf("\nStopping motors...\n");
    motor_stop();
    sleep_ms(1000);
    
    // Test 4: Turn right (motor1 faster)
    printf("\nTEST 4: Turning right for 2 seconds\n");
    motor_set_speed_rpm(40.0f, 20.0f);
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    for (int i = 0; i < 20; i++) {
        encoder_update_speed();
        
        float motor1_speed = encoder_get_speed_rpm(true);
        float motor2_speed = encoder_get_speed_rpm(false);
        
        printf("  Motor1: %.2f RPM | Motor2: %.2f RPM\n", motor1_speed, motor2_speed);
        sleep_ms(100);
    }
    
    // Stop
    printf("\nStopping motors...\n");
    motor_stop();
    sleep_ms(1000);
    
    // Test 5: Speed ramping test
    printf("\nTEST 5: Speed ramping from 0 to 60 RPM\n");
    for (int speed = 0; speed <= 60; speed += 10) {
        printf("  Setting speed to %d RPM\n", speed);
        motor_set_speed_rpm((float)speed, (float)speed);
        encoder_set_direction(true, true);
        encoder_set_direction(false, true);
        sleep_ms(1000);
        
        encoder_update_speed();
        float motor1_speed = encoder_get_speed_rpm(true);
        float motor2_speed = encoder_get_speed_rpm(false);
        printf("    Measured - Motor1: %.2f RPM | Motor2: %.2f RPM\n", 
               motor1_speed, motor2_speed);
    }
    
    // Final stop
    printf("\nStopping motors...\n");
    motor_stop();
    
    // Display total distance traveled
    sleep_ms(500);
    float total_motor1 = encoder_get_distance_mm(true) / 10.0f;   // mm to cm
    float total_motor2 = encoder_get_distance_mm(false) / 10.0f;  // mm to cm
    
    printf("\n========================================\n");
    printf("Test Complete!\n");
    printf("========================================\n");
    printf("Total Distance - Motor1: %.2f cm | Motor2: %.2f cm\n", 
           total_motor1, total_motor2);
    printf("Total Pulses - Motor1: %lu | Motor2: %lu\n",
           encoder_get_pulse_count(true), encoder_get_pulse_count(false));
    printf("\nResetting encoder distances...\n");
    
    encoder_reset(true, true);  // Reset both encoders
    
    printf("Test finished. System idle.\n");
    
    // Continuous monitoring loop
    printf("\nEntering continuous monitoring mode...\n");
    printf("(Motors stopped, showing encoder values every 2 seconds)\n\n");
    
    while (true) {
        encoder_update_speed();
        
        float motor1_speed = encoder_get_speed_rpm(true);
        float motor2_speed = encoder_get_speed_rpm(false);
        float motor1_dist = encoder_get_distance_mm(true) / 10.0f;
        float motor2_dist = encoder_get_distance_mm(false) / 10.0f;
        
        printf("Monitor - Motor1: %.2f RPM, %.2f cm | Motor2: %.2f RPM, %.2f cm\n", 
               motor1_speed, motor1_dist, motor2_speed, motor2_dist);
        
        sleep_ms(2000);
    }
    
    return 0;
}