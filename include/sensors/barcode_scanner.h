#ifndef BARCODE_SCANNER_H
#define BARCODE_SCANNER_H

#include "pico/stdlib.h"

// Barcode commands
typedef enum {
    BARCODE_NONE,
    BARCODE_LEFT,
    BARCODE_RIGHT,
    BARCODE_STOP,
    BARCODE_UTURN,
    BARCODE_UNKNOWN
} barcode_command_t;

// Barcode element structure for timing analysis
typedef struct {
    uint32_t duration_us;  // Duration in microseconds
    bool is_black;         // true = black bar, false = white space
} barcode_element_t;

// Code 39 character structure
typedef struct {
    char character;
    char pattern[10];      // 9 elements + null terminator (Wide=1, Narrow=0)
} code39_char_t;

/**
 * @brief Initialize barcode scanner system
 */
void barcode_scanner_init(void);

/**
 * @brief Detect if barcode pattern is starting
 * Looks for rapid black/white transitions indicating barcode presence
 * @return true if barcode pattern detected
 */
bool detect_barcode_start(void);

/**
 * @brief Scan and decode a complete barcode
 * Captures timing data and converts to Code 39 characters
 * @return Decoded barcode string (caller should not free)
 */
char* scan_and_decode_barcode(void);

/**
 * @brief Execute command based on barcode content
 * @param barcode_string Decoded barcode string
 * @return Command enum for robot action
 */
barcode_command_t get_barcode_command(const char* barcode_string);

/**
 * @brief Execute robot action based on barcode command
 * @param command Barcode command to execute
 */
void execute_barcode_command(barcode_command_t command);

/**
 * @brief Capture barcode timing as robot moves across it
 * @return true if sufficient timing data captured
 */
bool capture_barcode_timing(void);

/**
 * @brief Convert timing measurements to wide/narrow pattern
 * @return Pattern string (caller should not free)
 */
char* timing_to_pattern(void);

/**
 * @brief Decode Code 39 pattern to character
 * @param pattern 9-character pattern string
 * @return Decoded character or '?' if unknown
 */
char decode_character(const char* pattern);

/**
 * @brief Get string representation of barcode command
 * @param command Barcode command enum
 * @return String description of command
 */
const char* barcode_command_to_string(barcode_command_t command);

/**
 * @brief Check if we're currently scanning a barcode
 * @return true if barcode scan is in progress
 */
bool is_barcode_scanning(void);

#endif