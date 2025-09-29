#include "sensors/barcode_scanner.h"
#include "sensors/ir_sensor.h"
#include "common/robot_config.h"
#include <stdio.h>
#include <string.h>

// Global variables for barcode scanning
static barcode_element_t barcode_elements[MAX_BARCODE_ELEMENTS];
static int element_count = 0;
static bool scanning_in_progress = false;

// Code 39 character lookup table (partial - add more as needed)
static code39_char_t code39_table[] = {
    {'*', "100101001"},  // Start/Stop character
    {'A', "100100101"},
    {'B', "101100100"},
    {'C', "100110100"},
    {'L', "101001001"},  // LEFT command
    {'R', "100101010"},  // RIGHT command  
    {'S', "101010001"},  // STOP command
    {'U', "110001001"},  // U-TURN command
    {'\0', ""}           // End marker
};

void barcode_scanner_init(void) {
    // Initialize IR sensors (if not already done)
    ir_sensor_init();
    element_count = 0;
    scanning_in_progress = false;
    printf("Barcode scanner initialized\n");
}

bool detect_barcode_start(void) {
    static uint16_t previous_readings[5] = {0};
    static int reading_index = 0;
    static int transition_count = 0;
    
    uint16_t current = read_barcode_sensor();
    
    // Store current reading
    previous_readings[reading_index] = current;
    reading_index = (reading_index + 1) % 5;
    
    // Count transitions in recent readings
    transition_count = 0;
    for (int i = 0; i < 4; i++) {
        int diff = abs((int)previous_readings[i] - (int)previous_readings[(i+1)%5]);
        if (diff > BARCODE_TRANSITION_THRESHOLD) {
            transition_count++;
        }
    }
    
    // Barcode detected if we see multiple rapid transitions
    return transition_count >= 2;
}

bool capture_barcode_timing(void) {
    element_count = 0;
    scanning_in_progress = true;
    
    uint32_t start_time, current_time;
    bool current_state, previous_state;
    
    printf("Starting barcode capture...\n");
    
    // Wait for start of barcode (transition to black)
    while (!is_black_detected(read_barcode_sensor())) {
        sleep_ms(1);
    }
    
    previous_state = true;  // Started on black
    start_time = time_us_32();
    
    while (element_count < MAX_BARCODE_ELEMENTS) {
        current_state = is_black_detected(read_barcode_sensor());
        current_time = time_us_32();
        
        // Detect state change (black→white or white→black)
        if (current_state != previous_state) {
            // Record the duration of the previous element
            barcode_elements[element_count].duration_us = current_time - start_time;
            barcode_elements[element_count].is_black = previous_state;
            element_count++;
            
            start_time = current_time;
            previous_state = current_state;
        }
        
        // Stop if we've been on white for too long (end of barcode)
        if (!current_state && (current_time - start_time) > BARCODE_END_TIMEOUT_US) {
            break;
        }
        
        sleep_us(100);  // Sample every 100 microseconds
    }
    
    scanning_in_progress = false;
    printf("Captured %d barcode elements\n", element_count);
    return element_count >= 18;  // Minimum for valid Code 39 (start + 1 char + stop)
}

char* timing_to_pattern(void) {
    static char pattern[200];
    
    if (element_count < 9) {
        pattern[0] = '\0';
        return pattern;
    }
    
    // Calculate average duration to determine wide vs narrow
    uint32_t total_duration = 0;
    for (int i = 0; i < element_count; i++) {
        total_duration += barcode_elements[i].duration_us;
    }
    uint32_t avg_duration = total_duration / element_count;
    
    // Convert to pattern string
    int pattern_index = 0;
    for (int i = 0; i < element_count && pattern_index < 199; i++) {
        // Wide = duration > average, Narrow = duration <= average
        pattern[pattern_index++] = (barcode_elements[i].duration_us > avg_duration) ? '1' : '0';
    }
    pattern[pattern_index] = '\0';
    
    return pattern;
}

char decode_character(const char* pattern) {
    for (int i = 0; code39_table[i].character != '\0'; i++) {
        if (strncmp(pattern, code39_table[i].pattern, 9) == 0) {
            return code39_table[i].character;
        }
    }
    return '?';  // Unknown character
}

char* scan_and_decode_barcode(void) {
    static char decoded_string[50];
    decoded_string[0] = '\0';
    
    if (!capture_barcode_timing()) {
        strcpy(decoded_string, "ERROR: Insufficient data");
        return decoded_string;
    }
    
    char* pattern = timing_to_pattern();
    printf("Barcode pattern: %s\n", pattern);
    
    // Process pattern in groups of 9 (each Code 39 character)
    int decoded_length = 0;
    for (int i = 0; i + 8 < strlen(pattern); i += 9) {
        char char_pattern[10];
        strncpy(char_pattern, pattern + i, 9);
        char_pattern[9] = '\0';
        
        char decoded_char = decode_character(char_pattern);
        printf("Pattern %s → '%c'\n", char_pattern, decoded_char);
        
        if (decoded_char != '?' && decoded_length < 48) {
            decoded_string[decoded_length++] = decoded_char;
            decoded_string[decoded_length] = '\0';
        }
    }
    
    return decoded_string;
}

barcode_command_t get_barcode_command(const char* barcode_string) {
    if (strcmp(barcode_string, "*L*") == 0) {
        return BARCODE_LEFT;
    }
    else if (strcmp(barcode_string, "*R*") == 0) {
        return BARCODE_RIGHT;
    }
    else if (strcmp(barcode_string, "*S*") == 0) {
        return BARCODE_STOP;
    }
    else if (strcmp(barcode_string, "*U*") == 0) {
        return BARCODE_UTURN;
    }
    else if (strlen(barcode_string) == 0) {
        return BARCODE_NONE;
    }
    else {
        return BARCODE_UNKNOWN;
    }
}

void execute_barcode_command(barcode_command_t command) {
    printf("Executing command: %s\n", barcode_command_to_string(command));
    
    switch (command) {
        case BARCODE_LEFT:
            // TODO: Implement left turn
            printf("ACTION: Turn LEFT\n");
            break;
        case BARCODE_RIGHT:
            // TODO: Implement right turn  
            printf("ACTION: Turn RIGHT\n");
            break;
        case BARCODE_STOP:
            // TODO: Implement stop
            printf("ACTION: STOP\n");
            break;
        case BARCODE_UTURN:
            // TODO: Implement U-turn
            printf("ACTION: U-TURN\n");
            break;
        case BARCODE_NONE:
            printf("ACTION: Continue normal operation\n");
            break;
        case BARCODE_UNKNOWN:
            printf("ACTION: Unknown command - continue normal operation\n");
            break;
    }
}

const char* barcode_command_to_string(barcode_command_t command) {
    switch (command) {
        case BARCODE_NONE:    return "NONE";
        case BARCODE_LEFT:    return "LEFT";
        case BARCODE_RIGHT:   return "RIGHT"; 
        case BARCODE_STOP:    return "STOP";
        case BARCODE_UTURN:   return "U-TURN";
        case BARCODE_UNKNOWN: return "UNKNOWN";
        default:              return "INVALID";
    }
}

bool is_barcode_scanning(void) {
    return scanning_in_progress;
}