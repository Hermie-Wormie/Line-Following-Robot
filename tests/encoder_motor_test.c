/**
 * @file encoder_motor_test.c
 * @brief Comprehensive Motor and Encoder Test Program
 * 
 * Hardware Configuration:
 * - Motor 1 (Left):  PWM=GP8, DIR=GP9, Encoder=GP6
 * - Motor 2 (Right): PWM=GP10, DIR=GP11, Encoder=GP2
 * - Encoders: XCH206 (20 pulses/rev)
 * - Wheels: 65mm diameter
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#include "robot_config.h"

// Test configuration
#define TEST_SPEED_LOW      30.0f    // Low speed for testing (30%)
#define TEST_SPEED_MEDIUM   50.0f    // Medium speed (50%)
#define TEST_SPEED_HIGH     70.0f    // High speed (70%)
#define TEST_DURATION_MS    3000     // Duration for each test
#define MONITOR_INTERVAL_MS 100      // How often to print readings

// Helper function to print a nice separator
void print_separator(void) {
    printf("========================================\n");
}

// Helper function to print test header
void print_test_header(const char* test_name) {
    printf("\n");
    print_separator();
    printf("%s\n", test_name);
    print_separator();
}

// Monitor encoders for a specified duration
void monitor_encoders(uint32_t duration_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = start_time;
    
    printf("\nTime(s) | Motor1(L): RPM   Pulses   Dist(cm) | Motor2(R): RPM   Pulses   Dist(cm)\n");
    printf("--------|--------------------------------------|---------------------------------------\n");
    
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < duration_ms) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Print every MONITOR_INTERVAL_MS
        if ((current_time - last_print) >= MONITOR_INTERVAL_MS) {
            encoder_update_speed();  // Update speed calculations
            
            // Get encoder data
            float m1_rpm = encoder_get_speed_rpm(true);
            float m2_rpm = encoder_get_speed_rpm(false);
            
            uint32_t m1_pulses = encoder_get_pulse_count(true);
            uint32_t m2_pulses = encoder_get_pulse_count(false);
            
            float m1_dist = encoder_get_distance_mm(true) / 10.0f;  // mm to cm
            float m2_dist = encoder_get_distance_mm(false) / 10.0f;
            
            float elapsed = (current_time - start_time) / 1000.0f;
            
            printf("%6.1f  | %6.1f   %6lu   %7.1f        | %6.1f   %6lu   %7.1f\n",
                   elapsed, m1_rpm, m1_pulses, m1_dist, m2_rpm, m2_pulses, m2_dist);
            
            last_print = current_time;
        }
        
        sleep_ms(10);  // Small delay to prevent CPU hogging
    }
}

// Test 1: Check encoder functionality without motors
void test_encoder_detection(void) {
    print_test_header("TEST 1: Encoder Detection (Manual Test)");
    printf("Instructions:\n");
    printf("  1. Motors are OFF\n");
    printf("  2. Manually spin each wheel SLOWLY\n");
    printf("  3. Watch for pulse counts increasing\n");
    printf("  4. Test runs for 10 seconds\n");
    printf("\nSpin the wheels now!\n\n");
    
    encoder_reset(true, true);  // Reset counters
    
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t last_m1 = 0, last_m2 = 0;
    
    while ((to_ms_since_boot(get_absolute_time()) - start) < 10000) {
        uint32_t m1_pulses = encoder_get_pulse_count(true);
        uint32_t m2_pulses = encoder_get_pulse_count(false);
        
        // Only print when counts change
        if (m1_pulses != last_m1 || m2_pulses != last_m2) {
            printf("Motor1 (Left) pulses: %lu | Motor2 (Right) pulses: %lu\n", 
                   m1_pulses, m2_pulses);
            last_m1 = m1_pulses;
            last_m2 = m2_pulses;
        }
        
        sleep_ms(50);
    }
    
    printf("\nResult:\n");
    if (last_m1 > 0) {
        printf("  ✓ Motor1 (Left) encoder WORKING - detected %lu pulses\n", last_m1);
    } else {
        printf("  ✗ Motor1 (Left) encoder NOT WORKING - check GP6 connection\n");
    }
    
    if (last_m2 > 0) {
        printf("  ✓ Motor2 (Right) encoder WORKING - detected %lu pulses\n", last_m2);
    } else {
        printf("  ✗ Motor2 (Right) encoder NOT WORKING - check GP2 connection\n");
    }
}

// Test 2: Forward motion
void test_forward_motion(void) {
    print_test_header("TEST 2: Forward Motion at 50% Speed");
    printf("Both motors should spin forward at equal speed\n");
    printf("Encoders should show positive distance and RPM\n");
    
    encoder_reset(true, true);
    encoder_set_direction(true, true);   // Left forward
    encoder_set_direction(false, true);  // Right forward
    
    motor_set_speed(TEST_SPEED_MEDIUM, TEST_SPEED_MEDIUM);
    monitor_encoders(TEST_DURATION_MS);
    motor_stop();
    
    printf("\nTest complete - motors stopped\n");
    sleep_ms(1000);
}

// Test 3: Backward motion
void test_backward_motion(void) {
    print_test_header("TEST 3: Backward Motion at 50% Speed");
    printf("Both motors should spin backward at equal speed\n");
    printf("Encoders should show negative distance\n");
    
    encoder_reset(true, true);
    encoder_set_direction(true, false);   // Left backward
    encoder_set_direction(false, false);  // Right backward
    
    motor_set_speed(-TEST_SPEED_MEDIUM, -TEST_SPEED_MEDIUM);
    monitor_encoders(TEST_DURATION_MS);
    motor_stop();
    
    printf("\nTest complete - motors stopped\n");
    sleep_ms(1000);
}

// Test 4: Turn left
void test_turn_left(void) {
    print_test_header("TEST 4: Turn Left");
    printf("Left motor slower, right motor faster\n");
    printf("Right encoder should show higher RPM\n");
    
    encoder_reset(true, true);
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    motor_set_speed(TEST_SPEED_LOW, TEST_SPEED_MEDIUM);
    monitor_encoders(TEST_DURATION_MS);
    motor_stop();
    
    printf("\nTest complete - motors stopped\n");
    sleep_ms(1000);
}

// Test 5: Turn right
void test_turn_right(void) {
    print_test_header("TEST 5: Turn Right");
    printf("Left motor faster, right motor slower\n");
    printf("Left encoder should show higher RPM\n");
    
    encoder_reset(true, true);
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    motor_set_speed(TEST_SPEED_MEDIUM, TEST_SPEED_LOW);
    monitor_encoders(TEST_DURATION_MS);
    motor_stop();
    
    printf("\nTest complete - motors stopped\n");
    sleep_ms(1000);
}

// Test 6: Speed ramp test
void test_speed_ramp(void) {
    print_test_header("TEST 6: Speed Ramp Test");
    printf("Speed increases from 20%% to 80%% in 20%% steps\n");
    printf("RPM should increase proportionally\n\n");
    
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    for (int speed = 20; speed <= 80; speed += 20) {
        printf("\n--- Setting speed to %d%% ---\n", speed);
        encoder_reset(true, true);
        
        motor_set_speed((float)speed, (float)speed);
        monitor_encoders(2000);  // 2 seconds per speed
    }
    
    motor_stop();
    printf("\nTest complete - motors stopped\n");
    sleep_ms(1000);
}

// Test 7: Encoder accuracy test
void test_encoder_accuracy(void) {
    print_test_header("TEST 7: Encoder Accuracy Test");
    printf("Measuring 10 wheel rotations\n");
    printf("Expected distance per revolution: %.2f cm\n", 
           (WHEEL_DIAMETER_MM * 3.14159) / 10.0f);
    printf("Expected pulses per revolution: %d\n", ENCODER_PULSES_PER_REV);
    
    encoder_reset(true, true);
    encoder_set_direction(true, true);
    encoder_set_direction(false, true);
    
    motor_set_speed(TEST_SPEED_LOW, TEST_SPEED_LOW);  // Slow speed for accuracy
    
    // Monitor until we get approximately 200 pulses (10 revolutions)
    uint32_t target_pulses = 10 * ENCODER_PULSES_PER_REV;
    
    printf("\nWaiting for %lu pulses (10 revolutions)...\n", target_pulses);
    printf("This may take 10-20 seconds\n\n");
    
    while (encoder_get_pulse_count(true) < target_pulses || 
           encoder_get_pulse_count(false) < target_pulses) {
        encoder_update_speed();
        
        uint32_t m1_pulses = encoder_get_pulse_count(true);
        uint32_t m2_pulses = encoder_get_pulse_count(false);
        
        printf("Motor1: %lu pulses (%.1f revs) | Motor2: %lu pulses (%.1f revs)\r",
               m1_pulses, m1_pulses / (float)ENCODER_PULSES_PER_REV,
               m2_pulses, m2_pulses / (float)ENCODER_PULSES_PER_REV);
        
        sleep_ms(200);
    }
    
    motor_stop();
    printf("\n\n");
    
    // Final measurements
    uint32_t m1_pulses = encoder_get_pulse_count(true);
    uint32_t m2_pulses = encoder_get_pulse_count(false);
    float m1_dist = encoder_get_distance_mm(true) / 10.0f;
    float m2_dist = encoder_get_distance_mm(false) / 10.0f;
    
    float expected_dist = (WHEEL_DIAMETER_MM * 3.14159 * 10) / 10.0f;  // 10 revs in cm
    
    printf("Results:\n");
    printf("  Motor1 (Left):  %lu pulses, %.2f cm traveled\n", m1_pulses, m1_dist);
    printf("  Motor2 (Right): %lu pulses, %.2f cm traveled\n", m2_pulses, m2_dist);
    printf("  Expected:       %lu pulses, %.2f cm traveled\n", target_pulses, expected_dist);
    printf("\n");
    
    float m1_error = ((float)m1_pulses - target_pulses) / target_pulses * 100.0f;
    float m2_error = ((float)m2_pulses - target_pulses) / target_pulses * 100.0f;
    
    printf("  Motor1 error: %.1f%%\n", m1_error);
    printf("  Motor2 error: %.1f%%\n", m2_error);
    
    sleep_ms(1000);
}

// Main test program
int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║   COMPREHENSIVE ENCODER AND MOTOR TEST PROGRAM         ║\n");
    printf("╔════════════════════════════════════════════════════════╝\n");
    printf("║ Hardware Configuration:                                ║\n");
    printf("║   Motor 1 (Left):  GP8(PWM), GP9(DIR), GP6(Encoder)   ║\n");
    printf("║   Motor 2 (Right): GP10(PWM), GP11(DIR), GP2(Encoder) ║\n");
    printf("║   Encoders: 20 pulses/rev, Wheels: 65mm diameter      ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize motor controller
    printf("Initializing motor controller... ");
    if (!motor_init()) {
        printf("FAILED!\n");
        printf("ERROR: Cannot initialize motors. Check hardware.\n");
        while(1) { tight_loop_contents(); }
    }
    printf("OK\n");
    
    // Initialize encoders
    printf("Initializing wheel encoders... ");
    if (!encoder_init(MOTOR1_ENCODER_A_PIN, MOTOR2_ENCODER_A_PIN)) {
        printf("FAILED!\n");
        printf("ERROR: Cannot initialize encoders. Check connections.\n");
        while(1) { tight_loop_contents(); }
    }
    printf("OK\n");
    
    printf("\nSystem ready!\n");
    sleep_ms(1000);
    
    // Run all tests
    test_encoder_detection();   // Test 1: Manual encoder test
    test_forward_motion();      // Test 2: Forward
    test_backward_motion();     // Test 3: Backward
    test_turn_left();           // Test 4: Left turn
    test_turn_right();          // Test 5: Right turn
    test_speed_ramp();          // Test 6: Speed ramp
    test_encoder_accuracy();    // Test 7: Accuracy test
    
    // All tests complete
    print_separator();
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║     ALL TESTS COMPLETED!               ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("TROUBLESHOOTING GUIDE:\n");
    printf("----------------------\n");
    printf("No encoder pulses detected?\n");
    printf("  • Check Grove cable connections (GP6 for Motor1, GP2 for Motor2)\n");
    printf("  • Verify encoder power (3.3V or 5V depending on sensor)\n");
    printf("  • Ensure encoder is aligned with wheel disc\n");
    printf("  • Check for loose wiring\n\n");
    
    printf("Motors spin but no encoder pulses?\n");
    printf("  • Encoder disc may not be attached to wheel shaft\n");
    printf("  • Encoder sensor too far from disc slots\n");
    printf("  • Wrong GPIO pins configured\n\n");
    
    printf("Unequal speeds between motors?\n");
    printf("  • Normal mechanical variation\n");
    printf("  • Consider PID control for speed matching\n");
    printf("  • Check for friction or binding\n\n");
    
    printf("Entering continuous monitoring mode...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    // Continuous monitoring loop
    encoder_reset(true, true);
    motor_stop();
    
    while (true) {
        encoder_update_speed();
        
        uint32_t m1_pulses = encoder_get_pulse_count(true);
        uint32_t m2_pulses = encoder_get_pulse_count(false);
        float m1_rpm = encoder_get_speed_rpm(true);
        float m2_rpm = encoder_get_speed_rpm(false);
        float m1_dist = encoder_get_distance_mm(true) / 10.0f;
        float m2_dist = encoder_get_distance_mm(false) / 10.0f;
        
        printf("M1: %6lu pulses, %6.1f RPM, %7.1f cm | M2: %6lu pulses, %6.1f RPM, %7.1f cm\n",
               m1_pulses, m1_rpm, m1_dist, m2_pulses, m2_rpm, m2_dist);
        
        sleep_ms(1000);
    }
    
    return 0;
}