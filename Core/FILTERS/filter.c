//#include "filter.h"
//#include "../ICM42688P/icm42688p.h"
//#include "../HMC5883L/hmc5883l.h"
//#include <math.h>
//#include <string.h>
//#include <stdio.h>
//
//extern ICM42688P_HandleTypeDef icm;
//extern ICM42688P_Data_t icmData;
//
////extern UART_HandleTypeDef huart6;
//char uart_buf[200];
//
//void PT1_Init(PT1Filter *f, float cutoff_freq, float sample_rate)
//{
//    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
//    float dt = 1.0f / sample_rate;
//    f->alpha = dt / (rc + dt);
//    f->state = 0.0f;
//}
//
//float PT1_Update(PT1Filter *f, float input)
//{
//    f->state += f->alpha * (input - f->state);
//    return f->state;
//}
//
//void PT2_Init(PT2Filter *f, float cutoff_freq, float sample_rate)
//{
//    f->cutoff = cutoff_freq;
//    f->sample_rate = sample_rate;
//    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
//    f->alpha = sample_rate * rc / (sample_rate * rc + 1.0f);
//    f->state = f->prev_state = 0.0f;
//}
//
//float PT2_Update(PT2Filter *f, float input)
//{
//    f->state = f->state + f->alpha * (input - f->state);
//    return f->state;
//}
//
//void filters_init(filter_state_t *state)
//{
//    memset(state, 0, sizeof(filter_state_t)); // Initialize all states to 0
//    state->is_calibrated = 0;
//
//    // Initialize biquad and notch filters for accelerometer (5 Hz LPF, 80 Hz Notch)
//    for (int i = 0; i < 3; i++) {
//    	Biquad_InitLowPass(&state->accel_lpf[i], 10.0f, SAMPLE_RATE_HZ, 0.707f);
//    	Biquad_InitNotch(&state->accel_notch[i], 80.0f, SAMPLE_RATE_HZ, 1.0f);
//    	Biquad_InitLowPass(&state->gyro_lpf[i], 80.0f, SAMPLE_RATE_HZ, 0.707f);
//    	Biquad_InitLowPass(&state->gyro_lpf2[i], 25.0f, SAMPLE_RATE_HZ, 0.707f);  // Second biquad stage
//    	Biquad_InitNotch(&state->gyro_notch[i], 72.0f, SAMPLE_RATE_HZ, 1.0f);
//    	Biquad_InitNotch(&state->gyro_notch2[i], 109.0f, SAMPLE_RATE_HZ, 5.0f);
//
//    	PT1_Init(&state->gyro_pt1[i], 70.0f, SAMPLE_RATE_HZ);  // Example 60Hz cutoff
//    	PT2_Init(&state->gyro_pt2[i], 110.0f, SAMPLE_RATE_HZ);
//
//        PT1_Init(&state->mag_pt1[i], 60.0f, SAMPLE_RATE_HZ);       // 10 Hz PT1
//
//    }
//}
//
//void filters_apply_gyro(filtered_axises *gyro_data, filter_state_t *state)
//{
//    static uint8_t print_counter = 0;
//    const uint8_t print_interval = 70;
//
//	  ICM42688P_ReadData(&icm, &icmData);
//
//    print_counter++;
//
//    if (isnan(icmData.gyro[0]) || isnan(icmData.gyro[1]) || isnan(icmData.gyro[2])) {
//        if (print_counter >= print_interval) {
//            snprintf(uart_buf, sizeof(uart_buf),
//                     "Invalid Gyro DPS | X:%6.2f Y:%6.2f Z:%6.2f\r\n",
//					 icmData.gyro[0], icmData.gyro[1], icmData.gyro[2]);
//        }
//        gyro_data->x = gyro_data->y = gyro_data->z = 0;
//        print_counter = (print_counter >= print_interval) ? 0 : print_counter;
//        return;
//    }
//
//    // Apply bias correction (in dps now)
//    if (state->is_calibrated & 0x01) {
//    	icmData.gyro[0] -= state->gyro_bias[0];
//    	icmData.gyro[1] -= state->gyro_bias[1];
//    	icmData.gyro[2] -= state->gyro_bias[2];
//    }
//
//    // Apply PT2 + Biquad + Notch cascade
//    gyro_data->x = PT1_Update(&state->gyro_pt1[0], icmData.gyro[0]);
//    gyro_data->y = PT1_Update(&state->gyro_pt1[1], icmData.gyro[1]);
//    gyro_data->z = PT1_Update(&state->gyro_pt1[2], icmData.gyro[2]);
//
//    gyro_data->x = Biquad_Update(&state->gyro_lpf[0], gyro_data->x);
//    gyro_data->y = Biquad_Update(&state->gyro_lpf[1], gyro_data->y);
//    gyro_data->z = Biquad_Update(&state->gyro_lpf[2], gyro_data->z);
////
////    gyro_data->x = Biquad_Update(&state->gyro_notch[0], gyro_data->x);
////    gyro_data->y = Biquad_Update(&state->gyro_notch[1], gyro_data->y);
////    gyro_data->z = Biquad_Update(&state->gyro_notch[2], gyro_data->z);
//
//
//
//    if (print_counter >= print_interval) {
//    	snprintf(uart_buf, sizeof(uart_buf),
//    	         "Gyro Raw (dps): X:%6.2f Y:%6.2f Z:%6.2f\r\n"
//    	         "Gyro Filtered: X:%6.2f Y:%6.2f Z:%6.2f\r\n",
//				 icmData.gyro[0], icmData.gyro[1], icmData.gyro[2],
//    	         gyro_data->x, gyro_data->y, gyro_data->z);
//        print_counter = 0;
//    }
//}
//
//void filters_apply_accel(filtered_axises *accel_data, filter_state_t *state)
//{
//	  ICM42688P_ReadData(&icm, &icmData);
//
//    // Apply bias correction (already normalized)
//    if (state->is_calibrated & 0x02) {
//    	icmData.accel[0] -= state->accel_bias[0];
//    	icmData.accel[1] -= state->accel_bias[1];
//    	icmData.accel[2]-= state->accel_bias[2];
//    }
//
//    // Apply LPF + Notch cascade
//    accel_data->x = Biquad_Update(&state->accel_lpf[0], icmData.accel[0]);
//    accel_data->y = Biquad_Update(&state->accel_lpf[1], icmData.accel[1]);
//    accel_data->z = Biquad_Update(&state->accel_lpf[2], icmData.accel[2]);
//
//    accel_data->x = Biquad_Update(&state->accel_notch[0], accel_data->x);
//    accel_data->y = Biquad_Update(&state->accel_notch[1], accel_data->y);
//    accel_data->z = Biquad_Update(&state->accel_notch[2], accel_data->z);
//}
//
//void Biquad_InitLowPass(BiquadFilter* f, float cutoff_freq, float sample_rate, float Q)
//{
//    float w0 = 2.0f * M_PI * cutoff_freq / sample_rate;
//    float cos_w0 = cosf(w0);
//    float sin_w0 = sinf(w0);
//    float alpha = sin_w0 / (2.0f * Q);
//
//    float a0 = 1.0f + alpha;
//    f->b0 = (1.0f - cos_w0) / (2.0f * a0);
//    f->b1 = (1.0f - cos_w0) / a0;
//    f->b2 = (1.0f - cos_w0) / (2.0f * a0);
//    f->a1 = -2.0f * cos_w0 / a0;
//    f->a2 = (1.0f - alpha) / a0;
//
//    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
//}
//
//void Biquad_InitNotch(BiquadFilter* f, float center_freq, float sample_rate, float Q)
//{
//    float w0 = 2.0f * M_PI * center_freq / sample_rate;
//    float cos_w0 = cosf(w0);
//    float sin_w0 = sinf(w0);
//    float alpha = sin_w0 / (2.0f * Q);
//
//    float a0 = 1.0f + alpha;
//    f->b0 = 1.0f / a0;
//    f->b1 = -2.0f * cos_w0 / a0;
//    f->b2 = 1.0f / a0;
//    f->a1 = -2.0f * cos_w0 / a0;
//    f->a2 = (1.0f - alpha) / a0;
//
//    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
//}
//void filters_apply_mag(filtered_axises *mag_data, filter_state_t *state)
//{
//    HMC5883L_Data_t raw_data;
//    HMC5883L_ProcessMagData();
//    HMC5883L_ReadMag(&raw_data);
//
//    if (state->is_calibrated & 0x08) {
//        raw_data.mag_x -= state->mag_bias[0];
//        raw_data.mag_y -= state->mag_bias[1];
//        raw_data.mag_z -= state->mag_bias[2];
//    }
//
////    gyro_data->x = PT1_Update(&state->gyro_pt1[0], icmData.gyro[0]);
////    gyro_data->y = PT1_Update(&state->gyro_pt1[1], icmData.gyro[1]);
////    gyro_data->z = PT1_Update(&state->gyro_pt1[2], icmData.gyro[2]);
//
//    mag_data->x = PT1_Update(&state->mag_pt1[0], raw_data.mag_x);
//    mag_data->y = PT1_Update(&state->mag_pt1[1], raw_data.mag_y);
//    mag_data->z = PT1_Update(&state->mag_pt1[2], raw_data.mag_z);
//}
//
//float Biquad_Update(BiquadFilter* f, float input)
//{
//    float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
//
//    f->x2 = f->x1;
//    f->x1 = input;
//    f->y2 = f->y1;
//    f->y1 = output;
//
//    return output;
//}
