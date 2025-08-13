#include "complementary_filter.h"
#include <math.h>

#define ALPHA 0.85f

void Normalize_Vector(float* x, float* y, float* z) {
    float norm = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (norm > 0.0001f) {
        *x /= norm;
        *y /= norm;
        *z /= norm;
    }
}

static void Normalize_Quaternion(Quaternion* q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

static Quaternion Quaternion_Multiply(Quaternion a, Quaternion b) {
    Quaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

static void Integrate_Gyro(Quaternion* q, float gx, float gy, float gz, float dt) {
    Quaternion omega = {0, gx, gy, gz};
    Quaternion q_dot = Quaternion_Multiply(*q, omega);

    q_dot.w *= 0.5f;
    q_dot.x *= 0.5f;
    q_dot.y *= 0.5f;
    q_dot.z *= 0.5f;

    q->w += q_dot.w * dt;
    q->x += q_dot.x * dt;
    q->y += q_dot.y * dt;
    q->z += q_dot.z * dt;

    Normalize_Quaternion(q);
}

static Quaternion Accel_To_Quaternion(float ax, float ay, float az) {
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    Quaternion q;
    q.w = cr * cp;
    q.x = sr * cp;
    q.y = cr * sp;
    q.z = -sr * sp;  // yaw is ignored
    return q;
}

void ComplementaryFilter_Init(Quaternion* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void ComplementaryFilter_Update(Quaternion* q,
                                 float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt) {
	gx *= 0.0174533f; // Remove if gyro is already in rad/s
	    gy *= 0.0174533f;
	    gz *= 0.0174533f;

    Integrate_Gyro(q, gx, gy, gz, dt);

    Normalize_Vector(&ax, &ay, &az);
    Quaternion acc_q = Accel_To_Quaternion(ax, ay, az);
    q->w = ALPHA * q->w + (1.0f - ALPHA) * acc_q.w;
    q->x = ALPHA * q->x + (1.0f - ALPHA) * acc_q.x;
    q->y = ALPHA * q->y + (1.0f - ALPHA) * acc_q.y;
    q->z = ALPHA * q->z + (1.0f - ALPHA) * acc_q.z;

    Normalize_Quaternion(q);
}

void Quaternion_ToEuler(const Quaternion* q, float* roll, float* pitch) {
    *roll = atan2f(2.0f * (q->w*q->x + q->y*q->z), 1.0f - 2.0f * (q->x*q->x + q->y*q->y));
    *pitch = asinf(2.0f * (q->w*q->y - q->z*q->x));
}/*
 * complementary_filter.c
 *
 *  Created on: Aug 1, 2025
 *      Author: naitiktiwari
 */


