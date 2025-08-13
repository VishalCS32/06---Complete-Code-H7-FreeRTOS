/*
 * icm42688p_calibration.c
 *
 *  Created on: Aug 12, 2025
 *      Author: vishal
 */


#include "../ICM42688P/icm42688p.h"
#include <stdio.h>

#define CALIBRATION_SAMPLES  500
#define CALIBRATION_DELAY_MS 5

extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;
extern Struct_ICM42688P ICM42688P;

static int32_t acc_x_offset, acc_y_offset, acc_z_offset;

void ICM42688P_Calibrate(void)
{
    int64_t acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
    int64_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

    int16_t accel_raw[3], gyro_raw[3];

    printf("ICM42688P Calibration Started. Keep sensor still...\n");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        ICM42688P_Get6AxisRawData(accel_raw, gyro_raw);

        acc_x_sum += accel_raw[0];
        acc_y_sum += accel_raw[1];
        acc_z_sum += accel_raw[2];
        gyro_x_sum += gyro_raw[0];
        gyro_y_sum += gyro_raw[1];
        gyro_z_sum += gyro_raw[2];

        HAL_Delay(CALIBRATION_DELAY_MS);
    }

    acc_x_offset = (int32_t)(acc_x_sum / CALIBRATION_SAMPLES);
    acc_y_offset = (int32_t)(acc_y_sum / CALIBRATION_SAMPLES);
    acc_z_offset = (int32_t)(acc_z_sum / CALIBRATION_SAMPLES) - (int32_t)(32768 / 16); // Remove 1g offset in Z

    gyro_x_offset = (int32_t)(gyro_x_sum / CALIBRATION_SAMPLES);
    gyro_y_offset = (int32_t)(gyro_y_sum / CALIBRATION_SAMPLES);
    gyro_z_offset = (int32_t)(gyro_z_sum / CALIBRATION_SAMPLES);

    printf("Calibration Complete!\n");
    printf("Accel offsets: X=%ld, Y=%ld, Z=%ld\n", (long)acc_x_offset, (long)acc_y_offset, (long)acc_z_offset);
    printf("Gyro offsets : X=%ld, Y=%ld, Z=%ld\n", (long)gyro_x_offset, (long)gyro_y_offset, (long)gyro_z_offset);
}

void ICM42688P_GetCalibratedData(float *accel_g, float *gyro_dps)
{
    int16_t accel_raw[3], gyro_raw[3];

    ICM42688P_Get6AxisRawData(accel_raw, gyro_raw);

    accel_raw[0] -= acc_x_offset;
    accel_raw[1] -= acc_y_offset;
    accel_raw[2] -= acc_z_offset;

    gyro_raw[0] -= gyro_x_offset;
    gyro_raw[1] -= gyro_y_offset;
    gyro_raw[2] -= gyro_z_offset;

    accel_g[0] = ICM42688P_AccelRawToG(accel_raw[0]);
    accel_g[1] = ICM42688P_AccelRawToG(accel_raw[1]);
    accel_g[2] = ICM42688P_AccelRawToG(accel_raw[2]);

    gyro_dps[0] = ICM42688P_GyroRawToDPS(gyro_raw[0]);
    gyro_dps[1] = ICM42688P_GyroRawToDPS(gyro_raw[1]);
    gyro_dps[2] = ICM42688P_GyroRawToDPS(gyro_raw[2]);
}
