#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"

/* =========================================================
 *                SG90 SERVO DRIVER (PWM, 50 Hz)
 * ========================================================= */

typedef struct {
    uint8_t  gpio;
    uint     slice;
    uint     channel;
    uint16_t wrap;        // PWM top value (wrap)
    float    clkdiv;      // PWM clock divider
    int      min_us;      // e.g. 500–600 for SG90
    int      max_us;      // e.g. 2400–2500 for SG90
    int      min_deg;     // usually 0
    int      max_deg;     // usually 180
    bool     inited;
} servo_t;

static servo_t *g_servo_for_adapter = NULL; // used by servo_move adapter

static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool servo_init(servo_t *s, uint8_t gpio,
                int min_us, int max_us,
                int min_deg, int max_deg)
{
    if (!s) return false;

    // Configure PWM at 50 Hz with 1 µs tick
    s->gpio    = gpio;
    s->slice   = pwm_gpio_to_slice_num(gpio);
    s->channel = pwm_gpio_to_channel(gpio);
    s->clkdiv  = 125.0f;              // 125 MHz / 125 = 1 MHz → 1 tick = 1 µs
    s->wrap    = 20000 - 1;           // 20 ms period (50 Hz)

    s->min_us  = min_us;
    s->max_us  = max_us;
    s->min_deg = min_deg;
    s->max_deg = max_deg;

    gpio_set_function(gpio, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, s->clkdiv);
    pwm_config_set_wrap(&cfg, s->wrap);
    pwm_init(s->slice, &cfg, true);

    // Neutral
    int neutral = clamp_i((s->min_us + s->max_us) / 2, s->min_us, s->max_us);
    pwm_set_chan_level(s->slice, s->channel, (uint16_t)neutral);

    s->inited = true;
    g_servo_for_adapter = s;
    return true;
}

void servo_write_us(servo_t *s, int pulse_us) {
    if (!s || !s->inited) return;
    int us = clamp_i(pulse_us, s->min_us, s->max_us);
    uint16_t level = (uint16_t) clamp_i(us, 0, (int)s->wrap);
    pwm_set_chan_level(s->slice, s->channel, level);
}

void servo_write_deg(servo_t *s, int angle_deg) {
    if (!s || !s->inited) return;
    int a = clamp_i(angle_deg, s->min_deg, s->max_deg);
    int numer = (a - s->min_deg) * (s->max_us - s->min_us);
    int denom = (s->max_deg - s->min_deg) ? (s->max_deg - s->min_deg) : 1;
    int us = s->min_us + numer / denom;
    servo_write_us(s, us);
}

void servo_detach(servo_t *s) {
    if (!s || !s->inited) return;
    pwm_set_chan_level(s->slice, s->channel, 0);
    s->inited = false;
    if (g_servo_for_adapter == s) g_servo_for_adapter = NULL;
}

// Adapter for sweep helper
void servo_move(int angle_deg) {
    if (g_servo_for_adapter && g_servo_for_adapter->inited) {
        servo_write_deg(g_servo_for_adapter, angle_deg);
    }
}

/* =========================================================
 *               ULTRASONIC (YOUR ORIGINAL CODE)
 * ========================================================= */

// ---------------- Error Codes ----------------
typedef enum {
    ULTRA_OK = 0,
    ULTRA_EINVAL,
    ULTRA_EBUSY,
    ULTRA_ETIMEOUT,
    ULTRA_EHW,
    ULTRA_ENOINIT
} ultra_status_t;

// ---------------- Kalman Filter Structure ----------------
typedef struct {
    float x;          // Estimated state (filtered distance)
    float p;          // Estimation error covariance
    float q;          // Process variance (process noise)
    float r;          // Measurement variance (measurement noise)
    bool initialized;
} kalman_t;

// ---------------- Config & State ----------------
typedef struct {
    uint8_t trig_gpio;
    uint8_t echo_gpio;
    uint32_t min_quiet_us;     // Minimum quiet time (default: 60000 us = 60 ms)
    uint32_t max_range_mm;     // Max measurable range in mm (default: 4000 mm)
    bool inited;
    absolute_time_t last_trigger;
    ultra_status_t last_status;
    int rx_buf[20];            // static buffer per spec
    kalman_t kf;               // Kalman filter for this sensor
} ultra_t;

// ---------------- Kalman Filter Functions ----------------
void kalman_init(kalman_t *kf, float q, float r) {
    if (!kf) return;
    kf->x = 0.0f;           // Initial estimate
    kf->p = 1.0f;           // Initial estimation error
    kf->q = q;              // Process variance
    kf->r = r;              // Measurement variance
    kf->initialized = false;
}

float kalman_update(kalman_t *kf, float measurement) {
    if (!kf) return measurement;
    if (!kf->initialized) {
        kf->x = measurement;
        kf->p = 10.0f;  // Higher initial uncertainty
        kf->initialized = true;
        return kf->x;
    }
    float x_pred = kf->x;
    float p_pred = kf->p + kf->q;
    float k = p_pred / (p_pred + kf->r);
    kf->x = x_pred + k * (measurement - x_pred);
    kf->p = (1.0f - k) * p_pred;
    return kf->x;
}

void kalman_reset(kalman_t *kf) {
    if (!kf) return;
    kf->initialized = false;
    kf->x = 0.0f;
    kf->p = 1.0f;
}

// ---------------- Helper: Send Trigger Pulse ----------------
static inline void ultra_trigger_pulse(const ultra_t *u) {
    gpio_put(u->trig_gpio, 0);
    sleep_us(2);
    gpio_put(u->trig_gpio, 1);
    sleep_us(10);
    gpio_put(u->trig_gpio, 0);
}

// ---------------- Init ----------------
ultra_status_t ultra_init(ultra_t *u, uint8_t trig, uint8_t echo) {
    if (!u) return ULTRA_EINVAL;

    u->trig_gpio = trig;
    u->echo_gpio = echo;
    u->min_quiet_us = 60000;   // 60 ms
    u->max_range_mm = 4000;    // 4 m
    u->inited = true;
    u->last_trigger = make_timeout_time_us(0);
    u->last_status = ULTRA_OK;

    // Kalman defaults: Q=1.0, R=5.0
    kalman_init(&u->kf, 1.0f, 5.0f);

    gpio_init(trig);
    gpio_set_dir(trig, GPIO_OUT);
    gpio_put(trig, 0);

    gpio_init(echo);
    gpio_set_dir(echo, GPIO_IN);

    return ULTRA_OK;
}

// ---------------- Diagnostics ----------------
ultra_status_t ultra_last_status(const ultra_t *u) {
    if (!u || !u->inited) return ULTRA_ENOINIT;
    return u->last_status;
}

// ---------------- Core distance calculation ----------------
static ultra_status_t ultra_measure_once(ultra_t *u, int32_t *out_mm) {
    // Send trigger
    ultra_trigger_pulse(u);
    u->last_trigger = get_absolute_time();

    // Wait for echo HIGH (timeout ~30 ms)
    absolute_time_t wait_limit = make_timeout_time_us(30000);
    while (gpio_get(u->echo_gpio) == 0) {
        if (absolute_time_diff_us(get_absolute_time(), wait_limit) <= 0) {
            u->last_status = ULTRA_ETIMEOUT;
            return ULTRA_ETIMEOUT;
        }
    }
    absolute_time_t start = get_absolute_time();

    // Wait for echo LOW
    while (gpio_get(u->echo_gpio) == 1) {
        if (absolute_time_diff_us(get_absolute_time(), wait_limit) <= 0) {
            u->last_status = ULTRA_ETIMEOUT;
            return ULTRA_ETIMEOUT;
        }
    }
    absolute_time_t end = get_absolute_time();

    // Pulse width
    int64_t pulse_us = absolute_time_diff_us(start, end);

    // Convert to mm (sound speed ~343 m/s → 0.343 mm/µs; divide by 2 for round-trip)
    int32_t dist_mm = (int32_t)((pulse_us * 343) / 2000);

    if (dist_mm < 30) { // hardware blind zone
        u->last_status = ULTRA_EHW;
        return ULTRA_EHW;
    }

    if (out_mm) *out_mm = dist_mm;
    u->last_status = ULTRA_OK;
    return ULTRA_OK;
}

// ---------------- Get Raw (Unfiltered) Reading ----------------
ultra_status_t ultra_read_raw(ultra_t *u, int32_t *out_mm) {
    if (!u || !u->inited) return ULTRA_ENOINIT;

    // Enforce quiet time
    if (absolute_time_diff_us(get_absolute_time(), u->last_trigger) > -((int64_t)u->min_quiet_us)) {
        return ULTRA_EBUSY;
    }

    ultra_status_t st = ultra_measure_once(u, out_mm);
    if (st != ULTRA_OK) {
        // Retry once on spurious error
        st = ultra_measure_once(u, out_mm);
    }
    return st;
}

/* =========================================================
 *                  APPLICATION LOGIC
 * ========================================================= */

#define ULTRA_TRIG_GPIO   3
#define ULTRA_ECHO_GPIO   2
#define SERVO_GPIO       15
#define TOO_CLOSE_MM    100   // <-- Your new threshold

int main() {
    stdio_init_all();
    sleep_ms(2000);

    ultra_t sensor;
    ultra_status_t st = ultra_init(&sensor, ULTRA_TRIG_GPIO, ULTRA_ECHO_GPIO);
    if (st != ULTRA_OK) {
        printf("Ultrasonic init failed\n");
        return -1;
    }

    servo_t pan_servo;
    if (!servo_init(&pan_servo, SERVO_GPIO, 600, 2400, 0, 180)) {
        printf("Servo init failed\n");
        return -1;
    }

    printf("HC-SR04P + SG90 (30° sweep) ready (Too close ≤ %d mm)\n", TOO_CLOSE_MM);

    int dir = 1, a = 0;
    int32_t raw_dist;

    while (true) {
        // Sweep only 0° to 30°
        a += dir * 5;
        if (a >= 30) { a = 30; dir = -1; }
        if (a <= 0)  { a = 0;  dir = 1;  }
        servo_write_deg(&pan_servo, a);

        st = ultra_read_raw(&sensor, &raw_dist);
        if (st == ULTRA_OK) {
            float filtered = kalman_update(&sensor.kf, (float)raw_dist);
            int32_t kalman_dist = (int32_t)filtered;

            if (kalman_dist <= TOO_CLOSE_MM) {
                printf("Angle %2d° | Too close\n", a);
            } else {
                printf("Angle %2d° | Kalman distance: %4d mm\n", a, kalman_dist);
            }
        } else if (st == ULTRA_EHW) {
            // Hardware blind-zone still treated as "Too close"
            printf("Angle %2d° | Too close\n", a);
        }
        // Silence on ETIMEOUT/EBUSY/others, per your earlier preference.

        sleep_ms(120);  // adjust sweep speed
    }
}
