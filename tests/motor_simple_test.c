/**
 * @file motor_simple_test_working.c
 * @brief Working Motor Test - Guaranteed to Compile!
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_control.h"

#define TEST_DURATION_MS 3000

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║    MOTOR TEST - WORKING VERSION        ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize motors
    printf("Initializing motor controller...\n");
    if (!motor_init()) {
        printf("ERROR: Motor initialization failed!\n");
        while(1) { tight_loop_contents(); }
    }
    printf("✓ Motors initialized successfully\n\n");
    sleep_ms(500);
    
    // Test 1: Forward
    printf("Test 1: Moving FORWARD at 50%% speed\n");
    printf("        Both motors should spin forward\n");
    motor_set_speed(50.0f, 50.0f);
    sleep_ms(TEST_DURATION_MS);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 2: Backward
    printf("Test 2: Moving BACKWARD at 50%% speed\n");
    printf("        Both motors should spin backward\n");
    motor_set_speed(-50.0f, -50.0f);
    sleep_ms(TEST_DURATION_MS);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 3: Turn left
    printf("Test 3: Turning LEFT\n");
    printf("        Left motor slower, right motor faster\n");
    motor_set_speed(30.0f, 60.0f);
    sleep_ms(2000);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 4: Turn right
    printf("Test 4: Turning RIGHT\n");
    printf("        Left motor faster, right motor slower\n");
    motor_set_speed(60.0f, 30.0f);
    sleep_ms(2000);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 5: Pivot turn clockwise
    printf("Test 5: PIVOT TURN clockwise\n");
    printf("        Left forward, right backward\n");
    motor_set_speed(40.0f, -40.0f);
    sleep_ms(2000);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 6: Pivot turn counter-clockwise
    printf("Test 6: PIVOT TURN counter-clockwise\n");
    printf("        Left backward, right forward\n");
    motor_set_speed(-40.0f, 40.0f);
    sleep_ms(2000);
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    // Test 7: Speed ramp
    printf("Test 7: SPEED RAMP test (20%% to 80%%)\n");
    for (int speed = 20; speed <= 80; speed += 20) {
        printf("        Speed: %d%%\n", speed);
        motor_set_speed((float)speed, (float)speed);
        sleep_ms(1000);
    }
    motor_stop();
    printf("        Stopped.\n\n");
    sleep_ms(1000);
    
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║         ALL TESTS COMPLETE!            ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("Results:\n");
    printf("--------\n");
    printf("If motors spun: ✓ SUCCESS - Hardware working!\n");
    printf("If no movement: Check troubleshooting below\n\n");
    
    printf("TROUBLESHOOTING:\n");
    printf("----------------\n");
    printf("Motors didn't move?\n");
    printf("  1. Check power supply is ON (6-12V)\n");
    printf("  2. Check battery voltage (should be >6V)\n");
    printf("  3. Verify motor wires connected to M1A/M1B, M2A/M2B\n");
    printf("  4. Check Pico is powered (LED on)\n\n");
    
    printf("Only one motor works?\n");
    printf("  1. Swap motor connections to identify problem\n");
    printf("  2. Check for loose wire connections\n\n");
    
    printf("Motors spin wrong direction?\n");
    printf("  1. Swap motor wire polarity (M1A <-> M1B)\n");
    printf("  2. Or adjust in code with negative speeds\n\n");
    
    printf("System entering idle state...\n");
    
    // Idle loop
    while(1) {
        tight_loop_contents();
    }
    
    return 0;
}