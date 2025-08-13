//#ifndef __FILTER_H__
//#define __FILTER_H__
//
//#include "../ICM42688P/icm42688p.h"
//#include <stdint.h>
//#include <math.h>
//
//#define SAMPLE_RATE_HZ 1000.0f
//
//// Biquad filter structure (generic, can be used for LPF or Notch)
//typedef struct {
//    float b0, b1, b2, a1, a2;
//    float x1, x2; // previous inputs
//    float y1, y2; // previous outputs
//} BiquadFilter;
//
//typedef struct {
//    float state;
//    float prev_state;
//    float cutoff;
//    float sample_rate;
//    float alpha;
//} PT2Filter;
//
//typedef struct {
//    float state;
//    float alpha;
//} PT1Filter;
//
//typedef struct {
//    uint8_t is_calibrated;
//    float gyro_bias[3];
//    float accel_bias[3];
//    float mag_bias[3];
//
//    BiquadFilter accel_lpf[3];  // Biquad LPF for accel (5 Hz)
//    BiquadFilter gyro_lpf[3];   // Biquad LPF for gyro (50 Hz)
//    BiquadFilter gyro_lpf2[3];  // Second stage LPF per axis
//    BiquadFilter accel_notch[3]; // Notch filter for accel (80 Hz)
//    BiquadFilter gyro_notch[3];  // Notch filter for gyro (80 Hz)
//    BiquadFilter gyro_notch2[3];  // Notch filter for gyro (80 Hz)
//    PT1Filter gyro_pt1[3];  // New PT1 filter for gyro
//    PT2Filter gyro_pt2[3];  // NEW: PT2 Filter for gyro
//
//    PT1Filter mag_pt1[3];      // Low-pass filter for magnetometer
//
//
//
//} filter_state_t;
//typedef struct {
//    float x;
//    float y;
//    float z;
//} filtered_axises;
//
//void filters_init(filter_state_t *state);
//void filters_apply_gyro(filtered_axises *gyro_data, filter_state_t *state);
//void filters_apply_accel(filtered_axises *accel_data, filter_state_t *state);
//void filters_apply_mag(filtered_axises *mag_data, filter_state_t *state);
//
//void PT1_Init(PT1Filter *f, float cutoff_freq, float sample_rate);
//float PT1_Update(PT1Filter *f, float input);
//
//void PT2_Init(PT2Filter *f, float cutoff_freq, float sample_rate);
//float PT2_Update(PT2Filter *f, float input);
//
//// Biquad filter functions
//void Biquad_InitLowPass(BiquadFilter* f, float cutoff_freq, float sample_rate, float Q);
//void Biquad_InitNotch(BiquadFilter* f, float center_freq, float sample_rate, float Q);
//float Biquad_Update(BiquadFilter* f, float input);
//
//#endif /* __FILTERS_H__ */
