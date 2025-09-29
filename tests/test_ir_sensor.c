#include "sensors/ir_sensor.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    ir_sensor_init();
    
    printf("IR Sensor Test\n");
    printf("==============\n");
    
    while (true) {
        uint16_t line_val = read_line_sensor();
        uint16_t barcode_val = read_barcode_sensor();
        
        printf("Line: %d (%s, %.1f%%) | Barcode: %d (%s, %.1f%%)\n",
               line_val, is_black_detected(line_val) ? "BLACK" : "white",
               get_signal_strength(line_val),
               barcode_val, is_black_detected(barcode_val) ? "BLACK" : "white", 
               get_signal_strength(barcode_val));
               
        sleep_ms(200);
    }
    
    return 0;
}