#ifndef ICM42688P_CALIBRATION_H
#define ICM42688P_CALIBRATION_H

#include "../ICM42688P/icm42688p.h"

void ICM42688P_Calibrate(void);
void ICM42688P_GetCalibratedData(float *accel_g, float *gyro_dps);

#endif
