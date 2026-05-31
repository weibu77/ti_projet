#ifndef ICM_H
#define ICM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Ang2Rad 0.01745329252f
#define Rad2Ang 57.295779513f

typedef enum {
    BIQUAD_LOWPASS,
    BIQUAD_HIGHPASS,
    BIQUAD_BANDPASS_PEAK,
    BIQUAD_BANDSTOP_NOTCH,
} biquad_type;

typedef struct {
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_state;

typedef enum {
    ICM_YAW_MODE_RAW = 0,
    ICM_YAW_MODE_DEADZONE = 1,
    ICM_YAW_MODE_DEADZONE_LPF = 2,
    ICM_YAW_MODE_DEADZONE_LPF_BIAS = 3,
} ICM_YawMode;

extern float Pitch_a, Roll_a, Yaw_a;
extern float Pitch_g, Roll_g, Yaw_g;
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;
extern float Yaw_GyroAngle;
extern float Yaw_TotalAngle;
extern float Yaw_AngleLast;
extern int Yaw_RoundCount;
extern biquad_state adc_error;

void Filter_Init(void);
void ICM_ResetAttitude(void);

float ICM_YawFilterUpdate(float correctedGzDps, float dt_s);
void ICM_YawFilterReset(float yawDeg);
float ICM_YawFilterGetYawDeg(void);
float ICM_YawFilterGetInputRateDps(void);
float ICM_YawFilterGetFilteredRateDps(void);

void get_ICM_data(float dt_s);
float ICM_GetSampleDtMs(void);
float ICM_GetSampleHz(void);
void ICM_SetRuntimeBiasLearning(uint8_t enable);
float ICM_GetYawRuntimeBias(void);
float ICM_GetYawRateUsed(void);
void ICM_SetYawMode(ICM_YawMode mode);
ICM_YawMode ICM_GetYawMode(void);
const char *ICM_GetYawModeName(void);

float biquad(biquad_state *state, float data);
void biquad_filter_init(biquad_state *state, biquad_type type, int fs,
    float fc, float q_value);
float invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif
