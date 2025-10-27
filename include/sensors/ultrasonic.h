#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ======== Forward opaque types (implementation lives in ultrasonic.c) ======== */
typedef struct servo_t servo_t;
typedef struct ultra_t ultra_t;
typedef struct kalman_t kalman_t;

/* ======================= ULTRASONIC STATUS CODES ======================= */
typedef enum {
    ULTRA_OK = 0,
    ULTRA_EINVAL,
    ULTRA_EBUSY,
    ULTRA_ETIMEOUT,
    ULTRA_EHW,
    ULTRA_ENOINIT
} ultra_status_t;

/* ============================== SERVO API ============================== */
/**
 * Initialize an SG90-style servo on a GPIO using RP2040 PWM (50 Hz).
 * Typical SG90 calibration: min_us=600, max_us=2400, deg range 0..180.
 */
bool servo_init(servo_t *s, uint8_t gpio,
                int min_us, int max_us,
                int min_deg, int max_deg);

/** Write raw pulse width in microseconds (clamped to [min_us, max_us]). */
void servo_write_us(servo_t *s, int pulse_us);

/** Write an angle in degrees (clamped to [min_deg, max_deg]). */
void servo_write_deg(servo_t *s, int angle_deg);

/** Disable PWM on the servo channel (optional). */
void servo_detach(servo_t *s);

/**
 * Convenience adapter used by sweeping helpers: moves the “registered” servo
 * to the given angle (servo registered inside servo_init in ultrasonic.c).
 */
void servo_move(int angle_deg);

/* ============================ KALMAN API ============================ */
/** Initialize Kalman filter parameters (Q: process variance, R: measurement variance). */
void kalman_init(kalman_t *kf, float q, float r);

/** Push a measurement (in mm) and get the filtered estimate (in mm). */
float kalman_update(kalman_t *kf, float measurement);

/** Reset Kalman filter state. */
void kalman_reset(kalman_t *kf);

/* =========================== ULTRASONIC API =========================== */
/**
 * Initialize ultrasonic sensor (e.g., HC-SR04P).
 * trig: GPIO for trigger output, echo: GPIO for echo input.
 */
ultra_status_t ultra_init(ultra_t *u, uint8_t trig, uint8_t echo);

/** Get last status recorded by the driver. */
ultra_status_t ultra_last_status(const ultra_t *u);

/**
 * Take one raw (unfiltered) distance reading in millimeters.
 * Enforces quiet time; may return ULTRA_EBUSY/ETIMEOUT/EHW as per implementation.
 */
ultra_status_t ultra_read_raw(ultra_t *u, int32_t *out_mm);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ULTRASONIC_H */
