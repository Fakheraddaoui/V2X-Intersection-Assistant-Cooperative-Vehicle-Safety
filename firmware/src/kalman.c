#include "kalman.h"
#include <math.h>
#include <string.h>

void kf_init(kf_t *kf, float px, float py, float q_accel, float r_gps, float r_speed)
{
    memset(kf, 0, sizeof(*kf));
    kf->x[0] = px; kf->x[1] = py;
    for (int i = 0; i < KF_N; i++) kf->P[i][i] = (i < 2) ? 10.0f : 4.0f;
    kf->q_accel = q_accel;
    kf->r_gps = r_gps;
    kf->r_speed = r_speed;
}

void kf_predict(kf_t *kf, float dt)
{
    if (dt <= 0.0f) return;

    /* x = F x with F = [[I, dt*I],[0, I]] */
    kf->x[0] += kf->x[2] * dt;
    kf->x[1] += kf->x[3] * dt;

    /* P = F P F' + Q (discrete white-noise acceleration model) */
    float P[KF_N][KF_N];
    memcpy(P, kf->P, sizeof(P));
    for (int i = 0; i < 2; i++) {
        const int v = i + 2;
        const float Ppp = P[i][i], Ppv = P[i][v], Pvv = P[v][v];
        kf->P[i][i] = Ppp + 2.0f * dt * Ppv + dt * dt * Pvv;
        kf->P[i][v] = Ppv + dt * Pvv;
        kf->P[v][i] = kf->P[i][v];
    }
    const float q = kf->q_accel * kf->q_accel;
    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
    for (int i = 0; i < 2; i++) {
        const int v = i + 2;
        kf->P[i][i] += 0.25f * dt4 * q;
        kf->P[i][v] += 0.5f  * dt3 * q;
        kf->P[v][i] += 0.5f  * dt3 * q;
        kf->P[v][v] += dt2 * q;
    }
}

bool kf_update_gps(kf_t *kf, float zx, float zy, float gate)
{
    const float r = kf->r_gps * kf->r_gps;

    /* Innovation and its covariance (H selects position states) */
    const float yx = zx - kf->x[0];
    const float yy = zy - kf->x[1];
    const float Sx = kf->P[0][0] + r;
    const float Sy = kf->P[1][1] + r;

    /* Outlier rejection: per-axis Mahalanobis gate */
    const float d2 = (yx * yx) / Sx + (yy * yy) / Sy;
    if (gate > 0.0f && d2 > gate * gate) return false;

    for (int axis = 0; axis < 2; axis++) {
        const float y = (axis == 0) ? yx : yy;
        const float S = (axis == 0) ? Sx : Sy;
        for (int i = 0; i < KF_N; i++) {
            const float K = kf->P[i][axis] / S;
            kf->x[i] += K * y;
        }
        /* P = (I - K H) P for this scalar measurement */
        float col[KF_N];
        for (int i = 0; i < KF_N; i++) col[i] = kf->P[i][axis];
        for (int i = 0; i < KF_N; i++)
            for (int j = 0; j < KF_N; j++)
                kf->P[i][j] -= (col[i] / S) * kf->P[axis][j];
    }
    return true;
}

void kf_update_speed(kf_t *kf, float speed, float heading_rad)
{
    /* Convert scalar speed + heading into vx, vy pseudo-measurements */
    const float zvx = speed * cosf(heading_rad);
    const float zvy = speed * sinf(heading_rad);
    const float r = kf->r_speed * kf->r_speed;

    for (int axis = 2; axis < 4; axis++) {
        const float z = (axis == 2) ? zvx : zvy;
        const float y = z - kf->x[axis];
        const float S = kf->P[axis][axis] + r;
        float col[KF_N];
        for (int i = 0; i < KF_N; i++) col[i] = kf->P[i][axis];
        for (int i = 0; i < KF_N; i++) kf->x[i] += (col[i] / S) * y;
        for (int i = 0; i < KF_N; i++)
            for (int j = 0; j < KF_N; j++)
                kf->P[i][j] -= (col[i] / S) * kf->P[axis][j];
    }
}

float kf_speed(const kf_t *kf)
{
    return sqrtf(kf->x[2] * kf->x[2] + kf->x[3] * kf->x[3]);
}
