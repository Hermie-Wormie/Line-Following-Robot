#include "sensors/line_following.h"
#include "common/robot_config.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    line_following_init();
    
    printf("Line Following Test (Single Sensor)\n");
    printf("===================================\n");
    printf("Move line sensor over a black line\n");
    printf("Observe signal strength and state changes\n");
    printf("Press Ctrl+C to stop\n\n");
    
    while (true) {
        // Get complete line data
        line_data_t line_data = get_line_data();
        
        printf("Raw: %4d | Strength: %.2f | Level: %s | State: %s\n",
               line_data.raw_value,
               line_data.signal_strength,
               signal_level_to_string(line_data.level),
               line_state_to_string(line_data.state));
        
        // Visual signal strength indicator
        printf("Signal: [");
        int bars = (int)(line_data.signal_strength * 20);
        for (int i = 0; i < 20; i++) {
            if (i < bars) {
                printf("█");
            } else {
                printf("-");
            }
        }
        printf("] %.0f%%\n", line_data.signal_strength * 100);
        
        // Show line lost time if applicable
        if (line_data.state == LINE_LOST && line_data.lost_time_us > 0) {
            printf("Line lost for: %.1f ms\n", line_data.lost_time_us / 1000.0f);
        }
        
        // PID guidance (what motor control team would use)
        if (line_data.detected) {
            if (line_data.level == SIGNAL_STRONG) {
                printf("PID: Continue straight (strong signal)\n");
            } else if (line_data.level == SIGNAL_WEAK) {
                printf("PID: Slow down, on edge of line\n");
            }
        } else {
            printf("PID: Line lost - need search pattern\n");
        }
        
        printf("----------------------------------------\n");
        sleep_ms(300);
    }
    
    return 0;
}