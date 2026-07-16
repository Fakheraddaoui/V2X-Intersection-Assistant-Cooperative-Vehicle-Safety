#ifndef KALMAN_H
#define KALMAN_H
/*
 * Constant-velocity linear Kalman filter for planar vehicle state.
 * State x = [px, py, vx, vy]^T. Measurements: GPS position, wheel-odometry speed.
 * Pure C, no heap, deterministic — runs as a 200 Hz ThreadX task.
 */
#include <stdbool.h>

#define KF_N 4  /* state dim */

typedef struct {
    float x[KF_N];          /* state */
    float P[KF_N][KF_N];    /* covariance */
    float q_accel;          /* process noise: accel std (m/s^2) */
    float r_gps;            /* GPS position noise std (m) */
    float r_speed;          /* odometry speed noise std (m/s) */
} kf_t;

void kf_init(kf_t *kf, float px, float py, float q_accel, float r_gps, float r_speed);
void kf_predict(kf_t *kf, float dt);
/* Reject measurements with Mahalanobis distance > gate; returns false if rejected */
bool kf_update_gps(kf_t *kf, float zx, float zy, float gate);
void kf_update_speed(kf_t *kf, float speed, float heading_rad);
float kf_speed(const kf_t *kf);

#endif
