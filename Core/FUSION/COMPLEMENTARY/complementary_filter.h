#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <stdint.h>

typedef struct {
    float w, x, y, z;
} Quaternion;

void ComplementaryFilter_Init(Quaternion* q);
void ComplementaryFilter_Update(Quaternion* q,
                                 float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt);
void Quaternion_ToEuler(const Quaternion* q, float* roll, float* pitch);

#endif // COMPLEMENTARY_FILTER_H


