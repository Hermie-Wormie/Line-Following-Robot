#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "pico/stdlib.h"

// Line following states (realistic for single sensor)
typedef enum {
    LINE_DETECTED,    // Strong line signal detected
    LINE_EDGE,        // Weak line signal - on edge of line
    LINE_LOST         // No line signal detected
} line_state_t;

// Line signal strength levels
typedef enum {
    SIGNAL_NONE,      // 0-20% - completely off line
    SIGNAL_WEAK,      // 20-60% - edge of line
    SIGNAL_STRONG     // 60-100% - clearly on line
} line_signal_level_t;

// Line data structure (honest about single sensor capabilities)
typedef struct {
    float signal_strength;     // 0.0 to 1.0 (how much black detected)
    bool detected;             // true if any meaningful line signal
    line_state_t state;        // Current line state
    line_signal_level_t level; // Signal strength category
    uint16_t raw_value;        // Raw sensor reading
    uint32_t lost_time_us;     // Time since line was lost (0 if detected)
} line_data_t;

/**
 * @brief Initialize line following system
 */
void line_following_init(void);

/**
 * @brief Get current line data with all information
 * @return line_data_t structure with complete line status
 */
line_data_t get_line_data(void);

/**
 * @brief Get current line signal strength (0.0 to 1.0)
 * @return Signal strength where 0.0 = no line, 1.0 = strong line
 */
float get_line_signal_strength(void);

/**
 * @brief Check if line is currently detected
 * @return true if meaningful line signal detected
 */
bool is_line_detected(void);

/**
 * @brief Get current line state
 * @return line_state_t enum value
 */
line_state_t get_line_state(void);

/**
 * @brief Get signal strength level category
 * @return line_signal_level_t enum value
 */
line_signal_level_t get_signal_level(void);

/**
 * @brief Get time since line was lost
 * @return Time in microseconds, 0 if line currently detected
 */
uint32_t get_line_lost_time_us(void);

/**
 * @brief Convert signal strength to level category
 * @param strength Signal strength (0.0 to 1.0)
 * @return Corresponding signal level
 */
line_signal_level_t strength_to_level(float strength);

/**
 * @brief Get string representation of line state
 * @param state Line state enum
 * @return String description of state
 */
const char* line_state_to_string(line_state_t state);

/**
 * @brief Get string representation of signal level
 * @param level Signal level enum
 * @return String description of level
 */
const char* signal_level_to_string(line_signal_level_t level);

#endif