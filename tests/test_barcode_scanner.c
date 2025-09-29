#include "sensors/barcode_scanner.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    barcode_scanner_init();
    
    printf("Barcode Scanner Test\n");
    printf("===================\n");
    printf("Move barcode sensor over Code 39 barcodes\n");
    printf("Looking for patterns like *L*, *R*, *S*, *U*\n");
    printf("Press Ctrl+C to stop\n\n");
    
    while (true) {
        printf("Scanning for barcode...\n");
        
        // Wait for barcode detection
        while (!detect_barcode_start()) {
            sleep_ms(50);
        }
        
        printf("BARCODE DETECTED! Starting decode sequence...\n");
        
        // Scan and decode the barcode
        char* decoded = scan_and_decode_barcode();
        printf("Decoded barcode: '%s'\n", decoded);
        
        // Get command from barcode
        barcode_command_t command = get_barcode_command(decoded);
        printf("Command: %s\n", barcode_command_to_string(command));
        
        // Execute the command
        execute_barcode_command(command);
        
        printf("========================================\n");
        
        // Wait before scanning again
        sleep_ms(2000);
    }
    
    return 0;
}