#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_control.h"

int main() {
    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("Simple Motor Test (No Encoders)\n");
    printf("========================================\n\n");
    
    // Initialize motors
    printf("Initializing motors...\n");
    motor_init();
    sleep_ms(1000);
    
    // Test 1: Basic forward
    printf("\nTest 1: Motors forward at 50%% duty cycle for 3 seconds\n");
    motor_set_speed(50.0f, 50.0f);
    sleep_ms(3000);
    
    motor_stop();
    printf("Stopped.\n");
    sleep_ms(1000);
    
    // Test 2: Backward
    printf("\nTest 2: Motors backward at 50%% duty cycle for 3 seconds\n");
    motor_set_speed(-50.0f, -50.0f);
    sleep_ms(3000);
    
    motor_stop();
    printf("Stopped.\n");
    sleep_ms(1000);
    
    // Test 3: Turn left
    printf("\nTest 3: Turn left (Motor1=30%%, Motor2=60%%) for 2 seconds\n");
    motor_set_speed(30.0f, 60.0f);
    sleep_ms(2000);
    
    motor_stop();
    printf("Stopped.\n");
    sleep_ms(1000);
    
    // Test 4: Turn right
    printf("\nTest 4: Turn right (Motor1=60%%, Motor2=30%%) for 2 seconds\n");
    motor_set_speed(60.0f, 30.0f);
    sleep_ms(2000);
    
    motor_stop();
    printf("Stopped.\n");
    
    printf("\n========================================\n");
    printf("Motor test complete!\n");
    printf("If motors didn't move, check:\n");
    printf("1. Power supply connected?\n");
    printf("2. Motor wires connected correctly?\n");
    printf("3. GPIO pins correct in robot_config.h?\n");
    printf("4. Motor driver chip working?\n");
    printf("========================================\n");
    
    while(1) {
        tight_loop_contents();
    }
    
    return 0;
}