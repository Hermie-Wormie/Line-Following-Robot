#include "sensors/ir_sensor.h"
#include "common/robot_config.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    ir_sensor_init();
    
    printf("IR Sensor Basic Test\n");
    printf("===================\n");
    printf("White: %d | Black: %d | Threshold: %d\n", 
           WHITE_VALUE, BLACK_VALUE, THRESHOLD);
    printf("Move sensors over white and black surfaces\n");
    printf("Press Ctrl+C to stop\n\n");
    
    while (true) {
        // Read both sensors
        uint16_t line_raw = read_line_sensor_raw();
        uint16_t line_avg = read_line_sensor();
        uint16_t barcode_raw = read_barcode_sensor_raw();
        uint16_t barcode_avg = read_barcode_sensor();
        
        // Calculate signal strengths
        float line_strength = get_signal_strength(line_avg);
        float barcode_strength = get_signal_strength(barcode_avg);
        
        // Display readings
        printf("LINE    - Raw: %4d | Avg: %4d | %s | %.1f%%\n",
               line_raw, line_avg,
               is_black_detected(line_avg) ? "BLACK" : "WHITE",
               line_strength);
               
        printf("BARCODE - Raw: %4d | Avg: %4d | %s | %.1f%%\n",
               barcode_raw, barcode_avg,
               is_black_detected(barcode_avg) ? "BLACK" : "WHITE", 
               barcode_strength);
               
        printf("----------------------------------------\n");
        sleep_ms(300);
    }
    
    return 0;
}