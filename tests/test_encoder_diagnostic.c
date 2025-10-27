#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "common/robot_config.h"

/**
 * @file test_encoder_diagnostic.c
 * @brief Raw encoder signal diagnostic tool
 * 
 * This tool reads the encoder pins DIRECTLY without interrupts
 * to help diagnose hardware connection issues.
 */

void print_separator(void) {
    printf("\n========================================\n");
}

void print_header(const char *title) {
    print_separator();
    printf("  %s\n", title);
    print_separator();
}

/**
 * Test 1: Basic Pin State Reading
 */
void test_pin_states(void) {
    print_header("TEST 1: Raw Pin State Reading");
    
    printf("\nReading encoder pins for 5 seconds...\n");
    printf("Manually spin each wheel and watch for changes.\n\n");
    
    printf("Time | L-A | L-B | R-A | R-B\n");
    printf("-----|-----|-----|-----|-----\n");
    
    for (int i = 0; i < 50; i++) {
        bool left_a = gpio_get(MOTOR1_ENCODER_A_PIN);
        bool left_b = gpio_get(MOTOR1_ENCODER_B_PIN);
        bool right_a = gpio_get(MOTOR2_ENCODER_A_PIN);
        bool right_b = gpio_get(MOTOR2_ENCODER_B_PIN);
        
        printf("%4ds |  %d  |  %d  |  %d  |  %d\n", 
               i/10, left_a, left_b, right_a, right_b);
        
        sleep_ms(100);
    }
    
    printf("\n✓ Test 1 complete\n");
}

/**
 * Test 2: Transition Detection
 */
void test_transitions(void) {
    print_header("TEST 2: Transition Detection");
    
    printf("\nMonitoring for state changes (10 seconds)...\n");
    printf("Spin each wheel SLOWLY and count transitions.\n\n");
    
    bool prev_left_a = gpio_get(MOTOR1_ENCODER_A_PIN);
    bool prev_left_b = gpio_get(MOTOR1_ENCODER_B_PIN);
    bool prev_right_a = gpio_get(MOTOR2_ENCODER_A_PIN);
    bool prev_right_b = gpio_get(MOTOR2_ENCODER_B_PIN);
    
    int left_a_changes = 0;
    int left_b_changes = 0;
    int right_a_changes = 0;
    int right_b_changes = 0;
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 10000) {
        bool left_a = gpio_get(MOTOR1_ENCODER_A_PIN);
        bool left_b = gpio_get(MOTOR1_ENCODER_B_PIN);
        bool right_a = gpio_get(MOTOR2_ENCODER_A_PIN);
        bool right_b = gpio_get(MOTOR2_ENCODER_B_PIN);
        
        // Detect changes
        if (left_a != prev_left_a) {
            left_a_changes++;
            printf("[%.2fs] LEFT A changed: %d → %d\n", 
                   (to_ms_since_boot(get_absolute_time()) - start_time) / 1000.0f,
                   prev_left_a, left_a);
            prev_left_a = left_a;
        }
        
        if (left_b != prev_left_b) {
            left_b_changes++;
            printf("[%.2fs] LEFT B changed: %d → %d\n",
                   (to_ms_since_boot(get_absolute_time()) - start_time) / 1000.0f,
                   prev_left_b, left_b);
            prev_left_b = left_b;
        }
        
        if (right_a != prev_right_a) {
            right_a_changes++;
            printf("[%.2fs] RIGHT A changed: %d → %d\n",
                   (to_ms_since_boot(get_absolute_time()) - start_time) / 1000.0f,
                   prev_right_a, right_a);
            prev_right_a = right_a;
        }
        
        if (right_b != prev_right_b) {
            right_b_changes++;
            printf("[%.2fs] RIGHT B changed: %d → %d\n",
                   (to_ms_since_boot(get_absolute_time()) - start_time) / 1000.0f,
                   prev_right_b, right_b);
            prev_right_b = right_b;
        }
        
        sleep_us(1000);  // 1ms polling
    }
    
    printf("\n--- Transition Summary ---\n");
    printf("Left A:  %d transitions\n", left_a_changes);
    printf("Left B:  %d transitions\n", left_b_changes);
    printf("Right A: %d transitions\n", right_a_changes);
    printf("Right B: %d transitions\n", right_b_changes);
    
    printf("\n✓ Test 2 complete\n");
}

/**
 * Test 3: Voltage Level Check
 */
void test_voltage_levels(void) {
    print_header("TEST 3: Pin Voltage Stability");
    
    printf("\nChecking if pins are stuck HIGH or LOW...\n");
    printf("(Pins should toggle when wheel spins)\n\n");
    
    int left_a_high = 0, left_a_low = 0;
    int left_b_high = 0, left_b_low = 0;
    int right_a_high = 0, right_a_low = 0;
    int right_b_high = 0, right_b_low = 0;
    
    for (int i = 0; i < 1000; i++) {
        if (gpio_get(MOTOR1_ENCODER_A_PIN)) left_a_high++; else left_a_low++;
        if (gpio_get(MOTOR1_ENCODER_B_PIN)) left_b_high++; else left_b_low++;
        if (gpio_get(MOTOR2_ENCODER_A_PIN)) right_a_high++; else right_a_low++;
        if (gpio_get(MOTOR2_ENCODER_B_PIN)) right_b_high++; else right_b_low++;
        sleep_us(100);
    }
    
    printf("Left Encoder A:  HIGH=%d%%, LOW=%d%%\n", 
           left_a_high/10, left_a_low/10);
    printf("Left Encoder B:  HIGH=%d%%, LOW=%d%%\n",
           left_b_high/10, left_b_low/10);
    printf("Right Encoder A: HIGH=%d%%, LOW=%d%%\n",
           right_a_high/10, right_a_low/10);
    printf("Right Encoder B: HIGH=%d%%, LOW=%d%%\n",
           right_b_high/10, right_b_low/10);
    
    printf("\n✓ Stuck HIGH (100%%) = Disconnected or no power\n");
    printf("✓ Stuck LOW (0%%) = Short to ground or encoder issue\n");
    printf("✓ Normal = Values change when wheel spins\n");
    
    printf("\n✓ Test 3 complete\n");
}

/**
 * Diagnostic Report
 */
void print_diagnostic_report(void) {
    print_header("DIAGNOSTIC CHECKLIST");
    
    printf("\n1. PIN CONFIGURATION:\n");
    printf("   Left Encoder:  A=GP%d, B=GP%d\n", 
           MOTOR1_ENCODER_A_PIN, MOTOR1_ENCODER_B_PIN);
    printf("   Right Encoder: A=GP%d, B=GP%d\n",
           MOTOR2_ENCODER_A_PIN, MOTOR2_ENCODER_B_PIN);
    
    printf("\n2. WIRING CHECKLIST:\n");
    printf("   ☐ Encoder VCC connected to 3.3V or 5V\n");
    printf("   ☐ Encoder GND connected to ground\n");
    printf("   ☐ Encoder OUT A connected to correct GPIO\n");
    printf("   ☐ Encoder OUT B connected to correct GPIO\n");
    printf("   ☐ No loose connections\n");
    printf("   ☐ Encoder disc aligned with sensor\n");
    
    printf("\n3. COMMON PROBLEMS:\n");
    printf("   • No transitions = No power or disconnected\n");
    printf("   • Stuck at 0 or 1 = Wrong pin or bad connection\n");
    printf("   • Only A changes = B pin disconnected\n");
    printf("   • Only B changes = A pin disconnected\n");
    printf("   • Random noise = Encoder too far from disc\n");
    
    printf("\n4. WHAT TO TRY:\n");
    printf("   1. Check encoder power (LED should light up)\n");
    printf("   2. Verify GPIO pin numbers in robot_config.h\n");
    printf("   3. Check for loose jumper wires\n");
    printf("   4. Ensure encoder disc is clean\n");
    printf("   5. Check encoder gap (should be 1-3mm)\n");
    
    printf("\n5. EXPECTED VALUES:\n");
    printf("   • %d pulses per wheel revolution\n", ENCODER_PULSES_PER_REV);
    printf("   • Both A and B should change\n");
    printf("   • Changes should be synchronized (quadrature)\n");
    print_separator();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    print_header("ENCODER HARDWARE DIAGNOSTIC TOOL");
    printf("\nThis tool will help identify encoder hardware problems.\n");
    printf("Have a screwdriver ready to manually spin the wheels!\n");
    
    // Initialize GPIO pins for reading
    printf("\nInitializing encoder pins...\n");
    gpio_init(MOTOR1_ENCODER_A_PIN);
    gpio_init(MOTOR1_ENCODER_B_PIN);
    gpio_init(MOTOR2_ENCODER_A_PIN);
    gpio_init(MOTOR2_ENCODER_B_PIN);
    
    gpio_set_dir(MOTOR1_ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(MOTOR1_ENCODER_B_PIN, GPIO_IN);
    gpio_set_dir(MOTOR2_ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(MOTOR2_ENCODER_B_PIN, GPIO_IN);
    
    gpio_pull_up(MOTOR1_ENCODER_A_PIN);
    gpio_pull_up(MOTOR1_ENCODER_B_PIN);
    gpio_pull_up(MOTOR2_ENCODER_A_PIN);
    gpio_pull_up(MOTOR2_ENCODER_B_PIN);
    
    printf("Initialization complete!\n");
    
    sleep_ms(2000);
    
    // Run diagnostic tests
    test_voltage_levels();
    sleep_ms(1000);
    
    test_pin_states();
    sleep_ms(1000);
    
    test_transitions();
    sleep_ms(1000);
    
    print_diagnostic_report();
    
    print_header("DIAGNOSTIC COMPLETE");
    printf("\nAnalyze the results above to identify the problem.\n");
    printf("Fix hardware issues before running motor tests again.\n");
    print_separator();
    
    return 0;
}