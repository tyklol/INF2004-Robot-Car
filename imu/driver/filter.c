#include "filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float bx[32], by[32], bz[32];
static int   N = 0, idx = 0, count = 0;
static float sx = 0, sy = 0, sz = 0;

void imu_lpf_init(int window) {
    if (window < 1) window = 1;
    if (window > 32) window = 32;
    N = window; idx = 0; count = 0;
    sx = sy = sz = 0;
    memset(bx, 0, sizeof(bx));
    memset(by, 0, sizeof(by));
    memset(bz, 0, sizeof(bz));
}

void imu_lpf_push(float ax, float ay, float az) {
    // subtract oldest
    sx -= bx[idx]; sy -= by[idx]; sz -= bz[idx];
    // insert newest
    bx[idx] = ax;  by[idx] = ay;  bz[idx] = az;
    sx += ax;      sy += ay;      sz += az;
    // advance
    idx = (idx + 1) % N;
    if (count < N) count++;
}

void imu_lpf_get(float* fax, float* fay, float* faz) {
    float denom = (count > 0) ? (float)count : 1.0f;
    if (fax) *fax = sx / denom;
    if (fay) *fay = sy / denom;
    if (faz) *faz = sz / denom;
}

// Simple tilt from accel (no gyro): roll/pitch only
void imu_tilt_from_accel(float ax, float ay, float az,
                         float* roll_deg, float* pitch_deg) {
    // Protect against divide-by-zero
    float g = sqrtf(ax*ax + ay*ay + az*az) + 1e-6f;

    // Conventional aerospace-ish:
    // roll  = atan2(ay, az)
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    if (roll_deg)  *roll_deg  = roll  * (180.0f / (float)M_PI);
    if (pitch_deg) *pitch_deg = pitch * (180.0f / (float)M_PI);
}
