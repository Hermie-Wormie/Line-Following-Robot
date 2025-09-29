#include <stdio.h>
#include "sensors/ir_sensor.h"
#include "sensors/line_following.h"
#include "sensors/barcode_scanner.h"
#include "common/robot_config.h"

int main() {
    stdio_init_all();
    
    // Initialize all IR sensor systems
    ir_sensor_init();
    line_following_init();
    barcode_scanner_init();
    
    printf("Robot Main System Started\n");
    printf("=========================\n");
    printf("Line following and barcode scanning active\n");
    printf("Press Ctrl+C to stop\n\n");
    
    while (true) {
        // Get line following data
        line_data_t line_data = get_line_data();
        
        // Display line following status
        if (line_data.detected) {
            printf("LINE: %s | Pos: %+.2f | Error: %+.2f\n",
                   line_state_to_string(line_data.state),
                   line_data.position,
                   get_line_error());
                   
            // TODO: Send line error to PID controller
            // float pid_output = compute_pid(get_line_error());
            // set_motor_speeds(base_speed + pid_output, base_speed - pid_output);
            
        } else {
            printf("LINE: LOST - Searching...\n");
            // TODO: Implement line search pattern
        }
        
        // Check for barcode
        if (detect_barcode_start()) {
            printf("BARCODE DETECTED! Scanning...\n");
            
            char* barcode = scan_and_decode_barcode();
            barcode_command_t command = get_barcode_command(barcode);
            
            printf("Barcode: '%s' -> Command: %s\n", 
                   barcode, barcode_command_to_string(command));
            
            execute_barcode_command(command);
            
            // Wait after barcode processing
            sleep_ms(1000);
        }
        
        sleep_ms(100);  // Main control loop delay
    }
    
    return 0;
}